#!/usr/bin/env python3

# Copyright 2024 VoltBro
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import math
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from turtlebro_interfaces.action import Move

from turtlebro_actions.utils.motion_profile import compute_trapezoidal_speed


class _MoveGoalContext:
    """Хранит состояние текущей цели перемещения."""

    def __init__(
        self,
        goal_handle,
        total_distance: float,
        direction: float,
        start_x: float,
        start_y: float,
        max_speed: float,
        min_speed: float,
    ) -> None:
        self.goal_handle = goal_handle
        self.total_distance = total_distance
        self.direction = direction
        self.start_x = start_x
        self.start_y = start_y
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.result = None
        self.done_event = threading.Event()


class MoveServer(Node):
    """Action-сервер, который перемещает робота вперед или назад."""

    def __init__(self) -> None:
        super().__init__('move_server_node')
        self._callback_group = ReentrantCallbackGroup()
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = Odometry()
        self._odom_received = False
        self._odom_event = threading.Event()
        self._last_odom_time = time.monotonic()
        self.create_subscription(
            Odometry,
            '/odom',
            self.subscriber_odometry_cb,
            10,
            callback_group=self._callback_group,
        )

        self._active_goal_lock = threading.Lock()
        self._active_goal: _MoveGoalContext | None = None
        self._control_timer = self.create_timer(
            1.0 / 40.0,
            self._control_step,
            callback_group=self._callback_group,
        )

        self._action_server = ActionServer(
            self,
            Move,
            'action_move',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info('Запущен Action-сервер перемещения')

    def subscriber_odometry_cb(self, msg: Odometry) -> None:
        self.odom = msg
        self._odom_received = True
        self._odom_event.set()
        self._last_odom_time = time.monotonic()

    def goal_callback(self, goal_request: Move.Goal) -> GoalResponse:
        with self._active_goal_lock:
            if self._active_goal is not None:
                self.get_logger().warning('Уже выполняется цель перемещения, отклоняю новую')
                return GoalResponse.REJECT
        self.get_logger().info(
            f'Получена цель перемещения: расстояние={goal_request.goal:.3f}, '
            f'скорость={goal_request.speed:.3f}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Получен запрос на отмену перемещения')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        total_distance = float(abs(goal.goal))
        if math.isclose(total_distance, 0.0, abs_tol=1e-4):
            goal_handle.succeed()
            result = Move.Result()
            result.result = 0.0
            return result

        if not self._wait_for_odom(timeout=2.0):
            self.get_logger().error('Одометрия недоступна, отменяю цель перемещения')
            goal_handle.abort()
            result = Move.Result()
            result.result = 0.0
            return result

        start_pose_x = float(self.odom.pose.pose.position.x)
        start_pose_y = float(self.odom.pose.pose.position.y)
        direction = 1.0 if goal.goal >= 0.0 else -1.0

        requested_speed = abs(goal.speed)
        max_speed = requested_speed if requested_speed > 0.0 else 0.09
        min_speed = min(max(max_speed * 0.2, 0.02), max_speed)

        context = _MoveGoalContext(
            goal_handle,
            total_distance,
            direction,
            start_pose_x,
            start_pose_y,
            max_speed,
            min_speed,
        )

        with self._active_goal_lock:
            self._active_goal = context

        self.get_logger().info('Выполняю цель перемещения')

        while rclpy.ok():
            if context.done_event.wait(timeout=0.1):
                break

        if context.result is None:
            self.cmd_vel.publish(Twist())
            goal_handle.abort()
            return self._build_result(0.0, direction)

        return context.result

    def _control_step(self) -> None:
        with self._active_goal_lock:
            context = self._active_goal

        if context is None:
            return

        goal_handle = context.goal_handle

        if goal_handle.is_cancel_requested:
            self.get_logger().info('Цель перемещения отменена')
            distance_travelled = self._compute_distance(
                context.start_x, context.start_y, self.odom.pose.pose.position
            )
            distance_travelled = min(distance_travelled, context.total_distance)
            self._complete_goal(
                context,
                status='cancel',
                result=self._build_result(distance_travelled, context.direction),
            )
            return

        if self._odom_stale_seconds() > 5.0:
            self.get_logger().warning('Одометрия не обновляется >5с, отменяю цель перемещения')
            self._complete_goal(context, status='abort', result=self._build_result(0.0, context.direction))
            return

        distance_passed = self._compute_distance(
            context.start_x, context.start_y, self.odom.pose.pose.position
        )
        distance_passed = min(distance_passed, context.total_distance)

        feedback = Move.Feedback()
        feedback.feedback = float(context.direction * distance_passed)
        goal_handle.publish_feedback(feedback)

        if distance_passed >= context.total_distance - 1e-4:
            self.get_logger().info('Цель перемещения выполнена')
            self._complete_goal(
                context,
                status='success',
                result=self._build_result(distance_passed, context.direction),
            )
            return

        speed = compute_trapezoidal_speed(
            distance_passed,
            context.total_distance,
            context.max_speed,
            min_speed=context.min_speed,
        )
        cmd = Twist()
        cmd.linear.x = context.direction * speed
        self.cmd_vel.publish(cmd)

    def _complete_goal(self, context: _MoveGoalContext, status: str, result: Move.Result) -> None:
        if status == 'success':
            context.goal_handle.succeed()
        elif status == 'cancel':
            context.goal_handle.canceled()
        else:
            context.goal_handle.abort()

        self.cmd_vel.publish(Twist())
        context.result = result
        context.done_event.set()
        with self._active_goal_lock:
            if self._active_goal is context:
                self._active_goal = None

    def _compute_distance(self, start_x: float, start_y: float, current_pose) -> float:
        dx = start_x - float(current_pose.x)
        dy = start_y - float(current_pose.y)
        return math.sqrt(dx * dx + dy * dy)

    def _build_result(self, distance: float, direction: float) -> Move.Result:
        result = Move.Result()
        result.result = float(direction * distance)
        return result

    def _odom_stale_seconds(self) -> float:
        if not self._odom_received:
            return float('inf')
        return max(0.0, time.monotonic() - self._last_odom_time)

    def _wait_for_odom(self, timeout: float) -> bool:
        if self._odom_event.wait(timeout=timeout):
            return self._odom_received
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Сервер перемещения остановлен пользователем')
    finally:
        node.cmd_vel.publish(Twist())
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
