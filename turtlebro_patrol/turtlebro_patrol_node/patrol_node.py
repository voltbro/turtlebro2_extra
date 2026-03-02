#!/usr/bin/env python3

# Copyright 2026 VoltBro
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
"""ROS 2 patrol node for TurtleBro."""

from __future__ import annotations

import math
from itertools import cycle
from pathlib import Path
import signal
import threading
from typing import Any, Optional

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tf_transformations import quaternion_from_euler

try:  # Python 3.11+
    import tomllib
except ImportError:  # pragma: no cover
    import tomli as tomllib  # type: ignore[no-redef]

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover
    NavigateToPose = None  # type: ignore[assignment]

from turtlebro_patrol.msg import PatrolPoint
from turtlebro_patrol.srv import PatrolControlCallback, PatrolPointCallback

SUPPORTED_COMMANDS = {'start', 'pause', 'resume', 'home', 'shutdown'}


def normalize_control_command(command: Any) -> Optional[str]:
    """Normalize patrol command and return None for unsupported values."""
    if not isinstance(command, str):
        return None
    normalized = command.strip().lower()
    if normalized in SUPPORTED_COMMANDS:
        return normalized
    return None


class PatrolNode(Node):
    """Patrol controller that sends sequential Nav2 goals."""

    def __init__(self) -> None:
        super().__init__('turtlebro_patrol')

        if NavigateToPose is None:
            raise RuntimeError('Пакет nav2_msgs недоступен, патрулирование невозможно')

        self._state = 'wait'
        self._shutdown_requested = False
        self._patrolling_cycle = None
        self._active_goal_lock = threading.Lock()
        self._active_goal_handle = None
        self._pending_goal_point = None
        self._nav_ready_logged = False
        self._shutdown_lock = threading.Lock()
        self._shutdown_done = False

        self._config = self._load_config_file()
        self._current_point = self._config['home']

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 5)
        self._reached_point_pub = self.create_publisher(
            PatrolPoint,
            '/patrol_control/reached',
            5,
        )
        self.create_service(
            PatrolControlCallback,
            'patrol_control',
            self._control_service_cb,
        )

        callback_service = (
            self.declare_parameter('point_callback_service', '').get_parameter_value().string_value.strip()
        )
        self._callback_client = None
        if callback_service:
            self._callback_client = self.create_client(PatrolPointCallback, callback_service)
            self.get_logger().info(f'Ожидание callback service: {callback_service}')
            if not self._callback_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().warning(f'Callback service {callback_service} пока недоступен')
        else:
            self.get_logger().info('point_callback_service не задан, callback отключен')

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Патрульный узел запущен')
        self._loop_timer = self.create_timer(0.1, self._control_step)

    def _load_config_file(self) -> dict[str, Any]:
        default_path = str(Path(get_package_share_directory('turtlebro_patrol')) / 'data' / 'goals.toml')
        config_file = (
            self.declare_parameter('config_data_file', default_path)
            .get_parameter_value()
            .string_value
            .strip()
        )
        if not config_file:
            config_file = default_path

        self.get_logger().info(f'Загрузка файла патруля: {config_file}')
        with open(config_file, 'rb') as file:
            parsed = tomllib.load(file)

        home = parsed.get('home')
        if not isinstance(home, dict):
            raise RuntimeError('В конфиге отсутствует раздел [home]')

        patrolling = parsed.get('patrolling', [])
        if not isinstance(patrolling, list):
            raise RuntimeError('В конфиге patrolling должен быть списком')

        config = {
            'home': self._sanitize_point(home, fallback_name='home'),
            'patrolling': [self._sanitize_point(point, fallback_name=f'Goal{i + 1}') for i, point in enumerate(patrolling)],
        }
        return config

    def _sanitize_point(self, point: Any, *, fallback_name: str) -> dict[str, Any]:
        if not isinstance(point, dict):
            raise RuntimeError(f'Точка {fallback_name} имеет некорректный формат')
        pose = point.get('pose')
        if not isinstance(pose, dict):
            raise RuntimeError(f'Точка {fallback_name} не содержит pose')
        return {
            'name': str(point.get('name', fallback_name)),
            'pose': {
                'x': float(pose.get('x', 0.0)),
                'y': float(pose.get('y', 0.0)),
                'theta': float(pose.get('theta', 0.0)),
            },
        }

    def _control_service_cb(
        self,
        request: PatrolControlCallback.Request,
        response: PatrolControlCallback.Response,
    ) -> PatrolControlCallback.Response:
        command = normalize_control_command(request.command)
        if command is None:
            response.result = 'Command unrecognized'
            self.get_logger().warning(f'Неизвестная команда patrol_control: {request.command}')
            return response

        self.get_logger().info(f'Patrol command: {command}')
        self._cancel_active_goal()

        if command == 'shutdown':
            self._state = 'shutdown'
            self._shutdown_requested = True
            response.result = 'Ok, goodbye'
            return response

        if command in {'start', 'resume'}:
            self._state = 'patrol'
        if command in {'pause', 'home'}:
            self._state = 'wait'

        if command in {'start', 'resume', 'home'}:
            self._pending_goal_point = self._get_patrol_point(command)

        response.result = "Ok, let's do it"
        return response

    def _control_step(self) -> None:
        if self._shutdown_requested:
            self._shutdown_requested = False
            self.get_logger().info('Получена команда shutdown, завершаю патрулирование')
            self._stop_robot()
            rclpy.shutdown()
            return

        if self._pending_goal_point is None:
            return

        if not self._nav_client.server_is_ready():
            if not self._nav_ready_logged:
                self.get_logger().info('Ожидание action сервера navigate_to_pose')
                self._nav_ready_logged = True
            return

        self._nav_ready_logged = False
        goal_point = self._pending_goal_point
        self._pending_goal_point = None
        self._send_goal(goal_point)

    def _send_goal(self, point: dict[str, Any]) -> None:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(point['pose']['x'])
        goal_msg.pose.pose.position.y = float(point['pose']['y'])

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, math.radians(float(point['pose']['theta'])))
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f"Отправка цели патруля: {point['name']}")
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda result, target=point: self._goal_response_cb(result, target))

    def _goal_response_cb(self, future, point: dict[str, Any]) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Ошибка отправки цели {point['name']}: {exc}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning(f"Цель патруля отклонена: {point['name']}")
            return

        with self._active_goal_lock:
            self._active_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda result, target=point: self._goal_result_cb(result, target))

    def _goal_result_cb(self, future, point: dict[str, Any]) -> None:
        with self._active_goal_lock:
            self._active_goal_handle = None

        try:
            goal_result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Ошибка результата цели {point['name']}: {exc}")
            return

        if goal_result is None:
            self.get_logger().warning(f"Пустой результат цели {point['name']}")
            return

        status = int(goal_result.status)
        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f"Цель патруля отменена: {point['name']}")
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning(f"Цель патруля завершена статусом {status}: {point['name']}")
            return

        self.get_logger().info(f"Точка достигнута: {point['name']}")
        patrol_point = PatrolPoint()
        patrol_point.x = float(point['pose']['x'])
        patrol_point.y = float(point['pose']['y'])
        patrol_point.theta = int(round(float(point['pose']['theta'])))
        patrol_point.name = str(point['name'])
        self._reached_point_pub.publish(patrol_point)
        self._call_point_callback_service(patrol_point)

        if self._state == 'patrol':
            self._pending_goal_point = self._get_patrol_point('next')

    def _call_point_callback_service(self, patrol_point: PatrolPoint) -> None:
        if self._callback_client is None:
            return
        if not self._callback_client.service_is_ready():
            self.get_logger().warning('point_callback_service недоступен в момент вызова')
            return

        request = PatrolPointCallback.Request()
        request.patrol_point = patrol_point
        callback_future = self._callback_client.call_async(request)
        callback_future.add_done_callback(self._point_callback_done_cb)

    def _point_callback_done_cb(self, future) -> None:
        try:
            future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'Ошибка point_callback_service: {exc}')

    def _get_patrol_point(self, command: str) -> dict[str, Any]:
        if command in {'resume', 'current'}:
            return self._current_point

        if command == 'home':
            self._current_point = self._config['home']
            return self._current_point

        points = self._config['patrolling']
        if not points:
            self._current_point = self._config['home']
            return self._current_point

        if command == 'start':
            self._patrolling_cycle = cycle(points)
            self._current_point = next(self._patrolling_cycle)
            return self._current_point

        if command == 'next':
            if self._patrolling_cycle is None:
                self._patrolling_cycle = cycle(points)
            self._current_point = next(self._patrolling_cycle)
            return self._current_point

        return self._current_point

    def _cancel_active_goal(self, *, wait_timeout_sec: float = 0.0) -> None:
        with self._active_goal_lock:
            goal_handle = self._active_goal_handle
            self._active_goal_handle = None

        if goal_handle is None:
            return

        try:
            cancel_future = goal_handle.cancel_goal_async()
            if wait_timeout_sec > 0.0:
                try:
                    rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=wait_timeout_sec)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warning(f'Таймаут/ошибка ожидания отмены цели: {exc}')
                if cancel_future.done():
                    try:
                        cancel_future.result()
                    except Exception as exc:  # noqa: BLE001
                        self.get_logger().warning(f'Ошибка подтверждения отмены цели: {exc}')
            else:
                cancel_future.add_done_callback(self._goal_cancel_done_cb)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'Ошибка отмены цели патруля: {exc}')

    def _goal_cancel_done_cb(self, future) -> None:
        try:
            future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'Ошибка подтверждения отмены цели: {exc}')

    def _stop_robot(self) -> None:
        self._cmd_pub.publish(Twist())

    def on_shutdown(self) -> None:
        with self._shutdown_lock:
            if self._shutdown_done:
                return
            self._shutdown_done = True

        self.get_logger().info('Завершение patrol node')
        self._cancel_active_goal(wait_timeout_sec=1.0)
        for _ in range(3):
            self._stop_robot()


def _install_emergency_signal_handlers(node: PatrolNode) -> None:
    def _handle_signal(signum, _frame) -> None:
        try:
            node.get_logger().warning(f'Получен сигнал {signum}, аварийная остановка')
        except Exception:
            pass

        try:
            node.on_shutdown()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PatrolNode()
        _install_emergency_signal_handlers(node)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.on_shutdown()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
