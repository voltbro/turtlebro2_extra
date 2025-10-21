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
import subprocess
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple

from action_msgs.msg import GoalStatus
import cv2
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Float32MultiArray, Int16
from tf_transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
)
from turtlebro_interfaces.action import Move, Rotation

try:
    from turtlebro_speech.srv import Speech
except ImportError:  # pragma: no cover - optional dependency on robot image
    Speech = None  # type: ignore[assignment]

DEBUG = 1  # Включение/отключение отладочной печати


def _ensure_rclpy():
    if not rclpy.is_initialized():
        rclpy.init()


def _wait_for_message(node: Node, topic: str, msg_type: Any, timeout: Optional[float] = None):
    future: Future = Future()

    def _callback(msg):
        if not future.done():
            future.set_result(msg)

    subscription = node.create_subscription(msg_type, topic, _callback, 10)
    try:
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    finally:
        node.destroy_subscription(subscription)

    if not future.done():
        raise TimeoutError(f'На топик {topic} не поступило сообщение')
    return future.result()


class TurtleBro:
    """Робот TurtleBro с управлением движением, светодиодами, камерой и звуком."""

    def __init__(self):
        self._owns_context = not rclpy.is_initialized()
        _ensure_rclpy()
        self._node = rclpy.create_node('tb_py')
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self._subscriptions = []
        self._subscriptions.append(
            self._node.create_subscription(Odometry, '/odom', self.__subscriber_odometry_cb, 10)
        )
        self._subscriptions.append(
            self._node.create_subscription(
                Float32MultiArray, '/thermovisor', self.__subscriber_thermo_cb, 10
            )
        )
        self.vel_pub = self._node.create_publisher(Twist, '/cmd_vel', 10)

        # Начальные состояния
        self.odom = Odometry()
        self.thermo = Float32MultiArray()
        self.init_position_on_start = Odometry()
        self.odom_has_started = False

        self.u = Utility(self._node)  # Вспомогательные функции

        # Значения скорости
        self.linear_x_val = 0.09
        self.angular_z_val = 0.9
        self._action_timeout = 30.0
        self._move_client = ActionClient(self._node, Move, 'action_move')
        self._rotate_client = ActionClient(self._node, Rotation, 'action_rotate')
        self.__wait_for_action_server(self._move_client, 'action_move')
        self.__wait_for_action_server(self._rotate_client, 'action_rotate')

        self.wait_for_odom_to_start()

    def __del__(self):
        self.__shutdown()

    def __shutdown(self):
        """Корректно завершить работу с ROS 2."""
        if hasattr(self, 'vel_pub'):
            self.vel_pub.publish(Twist())
        if hasattr(self, '_executor'):
            self._executor.shutdown()
        if hasattr(self, '_spin_thread'):
            self._spin_thread.join(timeout=1.0)
        if hasattr(self, '_node') and self._node not in (None,):
            self._node.destroy_node()
        if self._owns_context and rclpy.ok():
            rclpy.shutdown()

    # Ожидание старта одометрии
    def wait_for_odom_to_start(self):
        while not self.odom_has_started and rclpy.ok():
            time.sleep(0.05)
        self.init_position_on_start = self.odom

    # Основные команды движения
    def forward(self, meters):
        assert meters > 0, 'Ошибка! Количество метров должно быть положительным'
        self.__move(meters)

    def backward(self, meters):
        assert meters > 0, 'Ошибка! Количество метров должно быть положительным'
        self.__move(-meters)

    def right(self, degrees):
        assert degrees > 0, 'Ошибка! Количество градусов должно быть положительным'
        self.__turn(-degrees)

    def left(self, degrees):
        assert degrees > 0, 'Ошибка! Количество градусов должно быть положительным'
        self.__turn(degrees)

    def goto(self, x, y, theta=0):
        self.__goto(x, y, theta)

    # Взаимодействие с Utility
    def call(self, name, button=24, *args, **kwargs):
        self.u.call(name, button, *args, **kwargs)

    def wait(self, duration):
        self.u.wait(duration)

    def color(self, col):
        self.u.color(col)

    def save_photo(self, name='robophoto'):
        self.u.photo(1, name)

    # Свойства для получения координат и данных с тепловизора
    @property
    def coords(self):
        angle_q = [
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w,
        ]
        (_, _, theta) = euler_from_quaternion(angle_q)
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        theta = math.degrees(theta)
        return x, y, theta

    @property
    def thermo_pixels(self):
        return self.thermo.data

    # Работа с камерой и звуком
    def get_photo(self):
        return self.u.photo(0, 'robophoto')

    def record(self, timeval=3, filename='turtlebro_sound'):
        self.u.record(timeval, filename)

    def say(self, text='Привет'):
        self.u.say(text)

    def play(self, filename):
        self.u.play(filename)

    def distance(self, angle=0):
        return self.u.distance(angle)

    # Настройка скорости движения
    def speed(self, value):
        assert isinstance(value, str), "'Скорость' должна быть fastest/fast/normal/slow/slowest"
        kp = 10
        speed_dict = {
            'fastest': 0.17,
            'fast': 0.12,
            'normal': 0.09,
            'slow': 0.04,
            'slowest': 0.01,
        }
        self.linear_x_val = speed_dict[value]
        self.angular_z_val = kp * self.linear_x_val

    # Callback функции для подписчиков
    def __subscriber_odometry_cb(self, msg):
        self.odom = msg
        if not self.odom_has_started:
            self.odom_has_started = True

    def __subscriber_thermo_cb(self, msg):
        self.thermo = msg

    # Основной метод движения через action
    def __move(self, meters):
        if DEBUG:
            print('Начало движения на метров:', meters)

        goal = Move.Goal()
        goal.goal = float(meters)
        goal.speed = float(abs(self.linear_x_val))

        result = self.__send_action_goal(
            self._move_client,
            goal,
            goal_name='move',
        )

        if DEBUG:
            print('Движение завершено. Проехано:', round(result.result, 2))
        return result.result

    # Навигация к точке
    def __goto(self, x, y, theta):
        heading = self.__get_turn_angle_to_point(x, y)
        distance = self.__get_distance_to_point(x, y)
        if DEBUG:
            print('поворот на:', heading)
            print('вперед на:', distance)
        self.__turn(math.degrees(heading))
        self.__move(distance)

    def __send_action_goal(
        self,
        client: ActionClient,
        goal_msg,
        *,
        goal_name: str,
        feedback_callback: Optional[Callable] = None,
    ):
        if feedback_callback is not None:
            goal_future = client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        else:
            goal_future = client.send_goal_async(goal_msg)

        goal_handle = self.__wait_for_future(
            goal_future,
            self._action_timeout,
            f'{goal_name} goal response',
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'Action {goal_name} отклонено или недоступно')

        result_future = goal_handle.get_result_async()
        goal_result = self.__wait_for_future(
            result_future,
            self._action_timeout,
            f'{goal_name} result',
        )
        if goal_result is None:
            raise RuntimeError(f'Action {goal_name} не вернул результат')
        if goal_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Action {goal_name} завершен со статусом {goal_result.status}')
        return goal_result.result

    def __wait_for_future(
        self,
        future: Future,
        timeout: float,
        description: str,
    ):
        start_time = time.perf_counter()
        while rclpy.ok() and not future.done():
            if timeout and (time.perf_counter() - start_time) > timeout:
                raise TimeoutError(f'{description} истек через {timeout:.1f} с')
            time.sleep(0.01)

        if not future.done():
            raise RuntimeError(f'{description} не завершился из-за остановки ROS')
        if future.cancelled():
            raise RuntimeError(f'{description} был отменен')

        exception = future.exception()
        if exception is not None:
            raise exception
        return future.result()

    def __wait_for_action_server(self, client: ActionClient, name: str, timeout: float = 5.0):
        if not client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(f'Action-сервер {name} недоступен спустя {timeout:.1f} с')
        self._node.get_logger().info(f'Подключение к Action-серверу {name} выполнено')

    # Метод поворота через action
    def __turn(self, degrees):
        if math.isclose(degrees, 0.0, abs_tol=0.1):
            return 0

        if DEBUG:
            print('Поворот на градусов:', degrees)

        goal = Rotation.Goal()
        goal.goal = int(round(degrees))
        goal.speed = float(abs(self.angular_z_val))

        result = self.__send_action_goal(
            self._rotate_client,
            goal,
            goal_name='rotate',
        )

        if DEBUG:
            print(f'Поворот завершен. Угол: {result.result}')
        return result.result

    def __get_turn_angle_to_point(self, x, y):
        angle_q = [
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w,
        ]
        (_, _, yaw) = euler_from_quaternion(angle_q)
        heading = -math.atan2(y, x)
        angle_to_turn = yaw - heading
        return angle_to_turn

    def __get_distance_to_point(self, x, y):
        return math.sqrt(
            (self.odom.pose.pose.position.x - x) ** 2
            + (self.odom.pose.pose.position.y - y) ** 2
        )

    # Прямое управление скоростью
    def linear_speed(self, v):
        vel = Twist()
        vel.linear.x = v
        self.vel_pub.publish(vel)
        if DEBUG:
            print('Еду с линейной скоростью:', v, 'м/с')

    def angular_speed(self, w):
        vel = Twist()
        w = math.radians(w)
        vel.angular.z = w
        self.vel_pub.publish(vel)
        if DEBUG:
            print('Поворачиваю с угловой скоростью:', w, 'радиан/с')


class TurtleNav(TurtleBro):
    """Робот с поддержкой навигации через Nav2 (navigate_to_pose)."""

    def __init__(self):
        super().__init__()
        self._nav_client = ActionClient(self._node, NavigateToPose, 'navigate_to_pose')

    def __goal_message_assemble(self, x, y, theta):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)

        q = quaternion_from_euler(0, 0, math.radians(float(theta)))
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]

        return goal

    def __goto(self, x, y, theta):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('Action-сервер NavigateToPose недоступен')

        goal = self.__goal_message_assemble(x, y, theta)
        send_future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('Цель NavigateToPose отклонена')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)


class Utility:
    """Вспомогательный класс с сенсорами, светодиодами, камерой и звуком."""

    def __init__(self, node: Node):
        self._node = node
        self.scan = LaserScan()
        self.names_of_func_to_call: Dict[int, Callable[..., None]] = {}
        self.args_of_func_to_call: Dict[int, Tuple[Any, ...]] = {}
        self.kwargs_of_func_to_call: Dict[int, Dict[str, Any]] = {}
        self._subscriptions = []
        self._subscriptions.append(
            self._node.create_subscription(LaserScan, '/scan', self.__subscriber_scan_cb, 10)
        )
        self._subscriptions.append(
            self._node.create_subscription(Int16, '/buttons', self.__subscriber_buttons_cb, 10)
        )
        self.colorpub = self._node.create_publisher(Int16, '/py_leds', 10)
        self.speech_client = None
        if Speech is not None:
            self.speech_client = self._node.create_client(Speech, 'festival_speech')

        time_counter = 0.0
        time_to_wait = 3.0
        while len(self.scan.ranges) <= 1 and rclpy.ok():
            time.sleep(0.1)
            time_counter += 0.1
            if time_counter > time_to_wait:
                break

        self.len_of_scan_ranges = len(self.scan.ranges) if self.scan.ranges else 0
        self.step_of_angles = self.len_of_scan_ranges / 360 if self.len_of_scan_ranges else 1
        self.retscan = [0] * 360
        print('Поехали!!!')

    def __del__(self):
        self.color('blue')

    # Callback функции
    def __subscriber_scan_cb(self, msg):
        self.scan = msg

    def __subscriber_buttons_cb(self, msg: Int16):
        try:
            if msg.data and msg.data in self.names_of_func_to_call:
                func = self.names_of_func_to_call[msg.data]
                args = self.args_of_func_to_call[msg.data]
                kwargs = self.kwargs_of_func_to_call[msg.data]
                func(*args, **kwargs)
                time.sleep(0.5)
        except BaseException:
            pass

    # Регистрация функций на кнопки
    def call(self, name, button=24, *args, **kwargs):
        self.names_of_func_to_call[button] = name
        self.args_of_func_to_call[button] = args
        self.kwargs_of_func_to_call[button] = kwargs

    def wait(self, duration=0):
        if duration == 0:
            try:
                while rclpy.ok():
                    time.sleep(0.1)
            except KeyboardInterrupt:
                pass
        else:
            time.sleep(duration)

    # Управление светодиодами
    def color(self, col):
        assert isinstance(
            col, str
        ), 'Имя цвета должно быть: red, green, blue, yellow, white или off'
        rgb = {'red': 1, 'green': 2, 'blue': 3, 'yellow': 4, 'white': 5, 'off': 6}
        shade = Int16()
        shade.data = int(rgb[col])
        self.colorpub.publish(shade)

    # Получение расстояния по лазеру
    def distance(self, angle):
        assert isinstance(angle, (int, float)), (
            'Угол должен быть числом от 0 до 359'
        )
        if not self.scan.ranges:
            return 0
        if angle == 0:
            if self.scan.ranges[0] != float('inf'):
                return self.scan.ranges[0]
            for i in range(-3, 3):
                if self.scan.ranges[i] != float('inf'):
                    return self.scan.ranges[i]
            return 0
        if angle < 360:
            idx = int(angle * self.step_of_angles)
            idx = max(0, min(idx, len(self.scan.ranges) - 1))
            return self.scan.ranges[idx]
        if angle == 360:
            for i in range(self.len_of_scan_ranges):
                k = int(i / self.step_of_angles) if self.step_of_angles else 0
                self.retscan[k] = self.scan.ranges[i]
            return self.retscan
        return None

    # Работа с камерой
    def photo(self, save, name):
        assert isinstance(name, str), 'Имя файла фото должно быть строкой'
        try:
            image_msg = _wait_for_message(
                self._node,
                '/front_camera/image_raw/compressed',
                CompressedImage,
                timeout=3.0,
            )
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            image_from_ros_camera = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if save:
                cv2.imwrite(f'/home/pi/{name}.jpg', image_from_ros_camera)
                if DEBUG:
                    print(f'Фото записано в /home/pi/{name}.jpg')
            else:
                return image_from_ros_camera
        except Exception as exc:  # noqa: BLE001
            print(exc)
        return None

    # Запись звука
    def record(self, timeval, filename, file_format='.wav'):
        assert timeval > 0 and isinstance(timeval, (float, int)), (
            'Временной интервал должен быть > 0'
        )
        process = subprocess.Popen([
            'arecord',
            '-D',
            'hw:1,0',
            '-f',
            'S16_LE',
            '-r',
            '48000',
            f'/home/pi/{filename}{file_format}',
        ])
        time.sleep(timeval)
        process.kill()

    # Произнесение текста
    def say(self, text):
        if not isinstance(text, str):
            text = str(text)
        if Speech is None or self.speech_client is None:
            raise RuntimeError('Сервис озвучивания недоступен: turtlebro_speech не установлен')
        while not self.speech_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError('Сервис озвучивания недоступен')
        request = Speech.Request()
        request.data = text
        future = self.speech_client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)

    # Воспроизведение файла
    def play(self, filename):
        assert filename, 'Файл для воспроизведения не задан'
        subprocess.Popen(['aplay', f'/home/pi/{filename}'])

    @staticmethod
    def __clamp(min_val, value, max_val):
        return max(min_val, min(value, max_val))
