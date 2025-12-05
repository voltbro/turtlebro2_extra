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
from tf_transformations import euler_from_quaternion

from turtlebro_interfaces.action import Rotation

from turtlebro_actions.utils.motion_profile import compute_trapezoidal_speed, normalize_angle


class _RotationGoalContext:
    """Хранит состояние текущей цели поворота."""

    def __init__(
        self,
        goal_handle,
        total_angle_rad: float,
        direction: float,
        max_speed: float,
        min_speed: float,
        prev_yaw: float,
    ) -> None:
        self.goal_handle = goal_handle
        self.total_angle_rad = total_angle_rad
        self.direction = direction
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.prev_yaw = prev_yaw
        self.accumulated_radians = 0.0
        self.result = None
        self.done_event = threading.Event()


class RotateServer(Node):
    """Action-сервер, который поворачивает робота вокруг оси Z."""

    def __init__(self) -> None:
        super().__init__('rotate_server_node')
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
        self._active_goal: _RotationGoalContext | None = None
        self._control_timer = self.create_timer(
            1.0 / 40.0,
            self._control_step,
            callback_group=self._callback_group,
        )

        self._action_server = ActionServer(
            self,
            Rotation,
            'action_rotate',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info('Запущен Action-сервер поворота')

    def subscriber_odometry_cb(self, msg: Odometry) -> None:
        self.odom = msg
        self._odom_received = True
        self._odom_event.set()
        self._last_odom_time = time.monotonic()

    def goal_callback(self, goal_request: Rotation.Goal) -> GoalResponse:
        with self._active_goal_lock:
            if self._active_goal is not None:
                self.get_logger().warning('Уже выполняется цель поворота, отклоняю новую')
                return GoalResponse.REJECT
        self.get_logger().info(
            f'Получена цель поворота: угол={goal_request.goal} скорость={goal_request.speed:.3f}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Получен запрос на отмену поворота')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        total_angle_deg = abs(goal.goal)
        if not self._wait_for_odom(timeout=2.0):
            self.get_logger().error('Одометрия недоступна, отменяю цель поворота')
            goal_handle.abort()
            return self._build_result(0.0)

        if total_angle_deg == 0:
            goal_handle.succeed()
            return self._build_result(0.0)

        total_angle_rad = math.radians(total_angle_deg)
        direction = 1.0 if goal.goal >= 0 else -1.0
        max_speed = abs(goal.speed) if goal.speed != 0.0 else 0.9
        min_speed = min(max(max_speed * 0.25, 0.1), max_speed)

        context = _RotationGoalContext(
            goal_handle,
            total_angle_rad,
            direction,
            max_speed,
            min_speed,
            prev_yaw=self._current_yaw(),
        )

        with self._active_goal_lock:
            self._active_goal = context

        self.get_logger().info('Выполняю цель поворота')

        while rclpy.ok():
            if context.done_event.wait(timeout=0.1):
                break

        if context.result is None:
            self.cmd_vel.publish(Twist())
            goal_handle.abort()
            return self._build_result(0.0)

        return context.result

    def _control_step(self) -> None:
        with self._active_goal_lock:
            context = self._active_goal

        if context is None:
            return

        goal_handle = context.goal_handle

        if goal_handle.is_cancel_requested:
            self.get_logger().info('Цель поворота отменена')
            self._complete_goal(
                context,
                status='cancel',
                result=self._build_result(context.accumulated_radians * context.direction),
            )
            return

        if self._odom_stale_seconds() > 5.0:
            self.get_logger().warning('Одометрия не обновляется >5с, отменяю цель поворота')
            self._complete_goal(context, status='abort', result=self._build_result(0.0))
            return

        current_yaw = self._current_yaw()
        delta = normalize_angle(current_yaw - context.prev_yaw)
        context.prev_yaw = current_yaw
        context.accumulated_radians += delta * context.direction
        context.accumulated_radians = max(
            min(context.accumulated_radians, context.total_angle_rad),
            0.0,
        )

        feedback = Rotation.Feedback()
        feedback.feedback = int(round(math.degrees(context.accumulated_radians * context.direction)))
        goal_handle.publish_feedback(feedback)

        if abs(context.accumulated_radians) >= context.total_angle_rad - math.radians(0.5):
            self.get_logger().info('Цель поворота выполнена')
            self._complete_goal(
                context,
                status='success',
                result=self._build_result(context.accumulated_radians * context.direction),
            )
            return

        speed = compute_trapezoidal_speed(
            abs(context.accumulated_radians),
            context.total_angle_rad,
            context.max_speed,
            min_speed=context.min_speed,
        )
        cmd = Twist()
        cmd.angular.z = context.direction * speed
        self.cmd_vel.publish(cmd)

    def _complete_goal(self, context: _RotationGoalContext, status: str, result: Rotation.Result) -> None:
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

    def _current_yaw(self) -> float:
        orientation = self.odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(
            [
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ]
        )
        return yaw

    def _build_result(self, radians_complete: float) -> Rotation.Result:
        result = Rotation.Result()
        result.result = int(round(math.degrees(radians_complete)))
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
    node = RotateServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Сервер поворота остановлен пользователем')
    finally:
        node.cmd_vel.publish(Twist())
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
