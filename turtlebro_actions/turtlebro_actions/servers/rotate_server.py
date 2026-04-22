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

from turtlebro_interfaces.action import Rotation

from turtlebro_actions.utils.motion_profile import (
    compute_trapezoidal_speed,
    normalize_angle,
    rotation_progress_from_start_rad,
)
from turtlebro_actions.utils.odom_helpers import current_yaw as odom_current_yaw

# Трапециевидный профиль угловой скорости и нижняя скорость (рад/с), если в цели speed=0
_ROTATE_DEFAULT_MAX_SPEED = 0.9
_ROTATE_MIN_SPEED_FRAC_OF_MAX = 0.1
_ROTATE_MIN_SPEED_FLOOR = 0.03
_ROTATE_DEFAULT_RAMP_TIME_SEC = 2.0

_CONTROL_RATE_HZ = 40.0
_CONTROL_DT = 1.0 / _CONTROL_RATE_HZ


class _RotationGoalContext:
    """Хранит состояние текущей цели поворота."""

    def __init__(
        self,
        goal_handle,
        total_angle_rad: float,
        direction: float,
        max_speed: float,
        min_speed: float,
        ramp_time_sec: float,
        start_yaw: float,
        prev_yaw: float,
        use_absolute_progress: bool,
    ) -> None:
        self.goal_handle = goal_handle
        self.total_angle_rad = total_angle_rad
        self.direction = direction
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.ramp_time_sec = ramp_time_sec
        self.start_yaw = start_yaw
        self.prev_yaw = prev_yaw
        self.use_absolute_progress = use_absolute_progress
        self.accumulated_radians = 0.0
        self.result: Rotation.Result | None = None
        self.done_event = threading.Event()


class RotateServer(Node):
    """Action-сервер, который поворачивает робота вокруг оси Z."""

    def __init__(self) -> None:
        super().__init__('rotate_server_node')
        self._callback_group = ReentrantCallbackGroup()

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('ramp_time_sec', _ROTATE_DEFAULT_RAMP_TIME_SEC)

        self.odom = Odometry()
        self._odom_received = False
        self._odom_event = threading.Event()
        self._odom_period_seconds = 0.1
        self._last_odom_time = time.monotonic()

        self._active_goal_lock = threading.Lock()
        self._active_goal: _RotationGoalContext | None = None

        # Ресурсы ленивые: создаются после первой цели, таймер работает только
        # пока цель активна (reset в execute, cancel в _complete_goal).
        self._resources_lock = threading.Lock()
        self.cmd_vel = None
        self._odom_subscription = None
        self._control_timer = None
        self._resources_ready = False

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
        now = time.monotonic()
        odom_dt = now - self._last_odom_time
        if 0.001 <= odom_dt <= 1.0:
            self._odom_period_seconds = 0.8 * self._odom_period_seconds + 0.2 * odom_dt
        self._last_odom_time = now

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
        with self._active_goal_lock:
            active_goal = self._active_goal
        if active_goal is not None and active_goal.goal_handle is not goal_handle:
            self.get_logger().warning('Запрос отмены поворота отклонен: активна другая цель')
            return CancelResponse.REJECT

        self.get_logger().info('Получен запрос на отмену поворота')
        if active_goal is not None and self.cmd_vel is not None:
            self.cmd_vel.publish(Twist())
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self._ensure_resources()
        goal = goal_handle.request
        total_angle_deg = abs(goal.goal)
        if total_angle_deg == 0:
            goal_handle.succeed()
            return self._build_result(0.0)

        if not self._wait_for_odom(timeout=2.0):
            self.get_logger().error('Одометрия недоступна, отменяю цель поворота')
            goal_handle.abort()
            return self._build_result(0.0)

        total_angle_rad = math.radians(total_angle_deg)
        direction = 1.0 if goal.goal >= 0 else -1.0
        max_speed = abs(goal.speed) if goal.speed != 0.0 else _ROTATE_DEFAULT_MAX_SPEED
        min_speed = min(
            max(max_speed * _ROTATE_MIN_SPEED_FRAC_OF_MAX, _ROTATE_MIN_SPEED_FLOOR),
            max_speed,
        )

        ramp_time_sec = float(
            self.get_parameter('ramp_time_sec').get_parameter_value().double_value
        )

        yaw0 = odom_current_yaw(self.odom)
        use_absolute = total_angle_rad <= math.pi + 1e-9
        context = _RotationGoalContext(
            goal_handle,
            total_angle_rad,
            direction,
            max_speed,
            min_speed,
            ramp_time_sec,
            start_yaw=yaw0,
            prev_yaw=yaw0,
            use_absolute_progress=use_absolute,
        )

        with self._active_goal_lock:
            self._active_goal = context

        self.get_logger().info('Выполняю цель поворота')
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

        yaw_now = odom_current_yaw(self.odom)
        if context.use_absolute_progress:
            context.accumulated_radians = rotation_progress_from_start_rad(
                context.start_yaw,
                yaw_now,
                context.direction,
                context.total_angle_rad,
                context.accumulated_radians,
            )
        else:
            delta = normalize_angle(yaw_now - context.prev_yaw)
            context.prev_yaw = yaw_now
            context.accumulated_radians += delta * context.direction
            context.accumulated_radians = max(context.accumulated_radians, 0.0)

        feedback = Rotation.Feedback()
        feedback.feedback = int(round(math.degrees(context.accumulated_radians * context.direction)))
        goal_handle.publish_feedback(feedback)

        if abs(context.accumulated_radians) >= context.total_angle_rad - math.radians(0.5):
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
            ramp_time_sec=context.ramp_time_sec,
        )
        remaining_angle_rad = max(context.total_angle_rad - context.accumulated_radians, 0.0)
        speed = min(speed, self._max_speed_for_remaining_angle(remaining_angle_rad))
        cmd = Twist()
        cmd.angular.z = context.direction * speed
        self.cmd_vel.publish(cmd)

    def _complete_goal(self, context: _RotationGoalContext, status: str, result: Rotation.Result) -> None:
        self.get_logger().info(f'Поворот завершён ({status}): {result.result}°')

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

    def _build_result(self, radians_complete: float) -> Rotation.Result:
        result = Rotation.Result()
        result.result = int(round(math.degrees(radians_complete)))
        return result

    def _odom_stale_seconds(self) -> float:
        if not self._odom_received:
            return float('inf')
        return max(0.0, time.monotonic() - self._last_odom_time)

    def _wait_for_odom(self, timeout: float) -> bool:
        if self._odom_stale_seconds() <= 1.0:
            return True
        self._odom_event.clear()
        if self._odom_event.wait(timeout=timeout):
            return self._odom_received
        return False

    def _max_speed_for_remaining_angle(self, remaining_angle_rad: float) -> float:
        if remaining_angle_rad <= 0.0:
            return 0.0
        odom_period = max(self._odom_period_seconds, _CONTROL_DT)
        return remaining_angle_rad / odom_period


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RotateServer()
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
