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

from turtlebro_actions.utils.linear_pid_controller import LinearPidController, LinearPidGains
from turtlebro_actions.utils.motion_profile import compute_trapezoidal_speed, normalize_angle
from turtlebro_actions.utils.odom_helpers import current_yaw, distance_xy

# Трапециевидный профиль линейной скорости и min_speed при перемещении (м/с)
_MOVE_DEFAULT_MAX_SPEED = 0.09
_MOVE_MIN_SPEED_FRAC_OF_MAX = 0.2
_MOVE_MIN_SPEED_FLOOR = 0.02
_MOVE_DEFAULT_RAMP_TIME_SEC = 2.0

_CONTROL_RATE_HZ = 40.0
_CONTROL_DT = 1.0 / _CONTROL_RATE_HZ


class _MoveLinearGoalContext:
    """Состояние цели линейного перемещения с коррекцией yaw по одометрии."""

    def __init__(
        self,
        goal_handle,
        total_distance: float,
        direction: float,
        start_x: float,
        start_y: float,
        yaw_ref: float,
        max_speed: float,
        min_speed: float,
        ramp_time_sec: float,
        linear_pid: LinearPidController,
    ) -> None:
        self.goal_handle = goal_handle
        self.total_distance = total_distance
        self.direction = direction
        self.start_x = start_x
        self.start_y = start_y
        self.yaw_ref = yaw_ref
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.ramp_time_sec = ramp_time_sec
        self.linear_pid = linear_pid
        self.result: Move.Result | None = None
        self.done_event = threading.Event()


class MoveLinearServer(Node):
    """Линейное перемещение вперёд/назад с коррекцией по yaw (orientation из /odom)."""

    def __init__(self) -> None:
        super().__init__('move_linear_server_node')
        self._callback_group = ReentrantCallbackGroup()

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('ramp_time_sec', _MOVE_DEFAULT_RAMP_TIME_SEC)
        self.declare_parameter('max_angular_z', 0.5)
        # PID по yaw → ω (в YAML: linear_pid.kp: 2.0 и т.д.)
        self.declare_parameter('linear_pid.kp', 5.0)
        self.declare_parameter('linear_pid.ki', 1.5)
        self.declare_parameter('linear_pid.kd', 0.0)
        self.declare_parameter('linear_pid.integral_limit', 0.0)

        self.odom = Odometry()
        self._odom_received = False
        self._odom_event = threading.Event()
        self._last_odom_time = time.monotonic()

        self._active_goal_lock = threading.Lock()
        self._active_goal: _MoveLinearGoalContext | None = None

        # Ресурсы ленивые: подписка /odom, publisher /cmd_vel, таймер — после первой цели.
        self._resources_lock = threading.Lock()
        self.cmd_vel = None
        self._odom_subscription = None
        self._control_timer = None
        self._resources_ready = False

        self._action_server = ActionServer(
            self,
            Move,
            'action_move',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info('Запущен Action-сервер линейного перемещения (коррекция по yaw)')

    def _ensure_resources(self) -> None:
        with self._resources_lock:
            if self._resources_ready:
                return
            odom_topic = str(self.get_parameter('odom_topic').get_parameter_value().string_value)
            self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
            self._odom_subscription = self.create_subscription(
                Odometry,
                odom_topic,
                self.subscriber_odometry_cb,
                10,
                callback_group=self._callback_group,
            )
            self._control_timer = self.create_timer(
                _CONTROL_DT,
                self._control_step,
                callback_group=self._callback_group,
            )
            self._control_timer.cancel()
            self._resources_ready = True
            self.get_logger().info(
                f'Ресурсы созданы: подписка на {odom_topic}, публикация /cmd_vel'
            )

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
            f'Получена цель линейного перемещения: расстояние={goal_request.goal:.3f}, '
            f'скорость={goal_request.speed:.3f}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        with self._active_goal_lock:
            active_goal = self._active_goal
        if active_goal is not None and active_goal.goal_handle is not goal_handle:
            self.get_logger().warning('Запрос отмены перемещения отклонен: активна другая цель')
            return CancelResponse.REJECT

        self.get_logger().info('Получен запрос на отмену перемещения')
        if active_goal is not None and self.cmd_vel is not None:
            self.cmd_vel.publish(Twist())
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self._ensure_resources()
        goal = goal_handle.request
        total_distance = float(abs(goal.goal))
        if math.isclose(total_distance, 0.0, abs_tol=1e-4):
            goal_handle.succeed()
            return self._build_result(0.0)

        if not self._wait_for_odom(timeout=2.0):
            self.get_logger().error('Одометрия недоступна, отменяю цель перемещения')
            goal_handle.abort()
            return self._build_result(0.0)

        start_pose_x = float(self.odom.pose.pose.position.x)
        start_pose_y = float(self.odom.pose.pose.position.y)
        yaw_ref = current_yaw(self.odom)
        direction = 1.0 if goal.goal >= 0.0 else -1.0

        requested_speed = abs(goal.speed)
        max_speed = requested_speed if requested_speed > 0.0 else _MOVE_DEFAULT_MAX_SPEED
        min_speed = min(
            max(max_speed * _MOVE_MIN_SPEED_FRAC_OF_MAX, _MOVE_MIN_SPEED_FLOOR),
            max_speed,
        )

        max_angular_z = float(self.get_parameter('max_angular_z').get_parameter_value().double_value)
        ramp_time_sec = float(
            self.get_parameter('ramp_time_sec').get_parameter_value().double_value
        )
        linear_pid = LinearPidController(self._read_linear_pid_gains(), output_limit=max_angular_z)

        context = _MoveLinearGoalContext(
            goal_handle,
            total_distance,
            direction,
            start_pose_x,
            start_pose_y,
            yaw_ref,
            max_speed,
            min_speed,
            ramp_time_sec,
            linear_pid,
        )

        with self._active_goal_lock:
            self._active_goal = context

        self.get_logger().info('Выполняю цель линейного перемещения')
        self._control_timer.reset()

        while rclpy.ok():
            if context.done_event.wait(timeout=0.1):
                break

        if context.result is None:
            self._safe_publish_stop()
            try:
                goal_handle.abort()
            except Exception:  # noqa: BLE001
                pass
            return self._build_result(0.0)

        return context.result

    def _read_linear_pid_gains(self) -> LinearPidGains:
        return LinearPidGains(
            kp=float(self.get_parameter('linear_pid.kp').get_parameter_value().double_value),
            ki=float(self.get_parameter('linear_pid.ki').get_parameter_value().double_value),
            kd=float(self.get_parameter('linear_pid.kd').get_parameter_value().double_value),
            integral_limit=float(
                self.get_parameter('linear_pid.integral_limit').get_parameter_value().double_value
            ),
        )

    def _linear_correction(self, context: _MoveLinearGoalContext) -> tuple[float, float]:
        """Возвращает (ω_z от PID, ошибку слежения yaw в рад).

        e = normalize_angle(yaw_ref − yaw): положительная — нужен поворот против часовой
        (положительный Twist.angular.z по REP-103).
        """
        e = normalize_angle(context.yaw_ref - current_yaw(self.odom))
        omega = context.linear_pid.step(e, _CONTROL_DT)
        return omega, e

    def _control_step(self) -> None:
        with self._active_goal_lock:
            context = self._active_goal

        if context is None:
            return

        if not rclpy.ok():
            context.done_event.set()
            return

        goal_handle = context.goal_handle

        if goal_handle.is_cancel_requested:
            self.get_logger().info('Цель перемещения отменена')
            distance_travelled = min(
                distance_xy(context.start_x, context.start_y, self.odom.pose.pose.position),
                context.total_distance,
            )
            self._complete_goal(
                context,
                status='cancel',
                result=self._build_result(context.direction * distance_travelled),
            )
            return

        if self._odom_stale_seconds() > 5.0:
            self.get_logger().warning('Одометрия не обновляется >5с, отменяю цель перемещения')
            self._complete_goal(context, status='abort', result=self._build_result(0.0))
            return

        distance_passed = min(
            distance_xy(context.start_x, context.start_y, self.odom.pose.pose.position),
            context.total_distance,
        )

        feedback = Move.Feedback()
        feedback.feedback = float(context.direction * distance_passed)
        goal_handle.publish_feedback(feedback)

        if distance_passed >= context.total_distance - 1e-4:
            self.get_logger().info('Цель перемещения выполнена')
            self._complete_goal(
                context,
                status='success',
                result=self._build_result(context.direction * distance_passed),
            )
            return

        speed = compute_trapezoidal_speed(
            distance_passed,
            context.total_distance,
            context.max_speed,
            min_speed=context.min_speed,
            ramp_time_sec=context.ramp_time_sec,
        )
        omega_z, yaw_err = self._linear_correction(context)

        cmd = Twist()
        cmd.linear.x = context.direction * speed
        cmd.angular.z = omega_z
        self.cmd_vel.publish(cmd)
        self.get_logger().info(
            f'линейное движение: отклонение yaw={yaw_err:.4f} рад ({math.degrees(yaw_err):.2f}°), '
            f'добавленная ω_z={omega_z:.4f} рад/с',
            throttle_duration_sec=1.0,
        )

    def _complete_goal(self, context: _MoveLinearGoalContext, status: str, result: Move.Result) -> None:
        try:
            if status == 'success':
                context.goal_handle.succeed()
            elif status == 'cancel':
                context.goal_handle.canceled()
            else:
                context.goal_handle.abort()
        except Exception:  # noqa: BLE001
            pass

        self._safe_publish_stop()
        context.result = result
        context.done_event.set()
        with self._active_goal_lock:
            if self._active_goal is context:
                self._active_goal = None
        if self._control_timer is not None:
            self._control_timer.cancel()

    def _safe_publish_stop(self) -> None:
        if self.cmd_vel is None:
            return
        try:
            self.cmd_vel.publish(Twist())
        except Exception:  # noqa: BLE001
            pass

    def _build_result(self, signed_distance: float) -> Move.Result:
        result = Move.Result()
        result.result = float(signed_distance)
        return result

    def _odom_stale_seconds(self) -> float:
        if not self._odom_received:
            return float('inf')
        return max(0.0, time.monotonic() - self._last_odom_time)

    def _wait_for_odom(self, timeout: float) -> bool:
        if self._odom_stale_seconds() <= 1.0:
            return True
        self._odom_event.clear()
        if not self._odom_event.wait(timeout=timeout):
            if not self._odom_received:
                self.get_logger().info(
                    'После lazy-подписки первое сообщение /odom не пришло за %.1f с' % timeout
                )
            return False
        return self._odom_received


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveLinearServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._safe_publish_stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
