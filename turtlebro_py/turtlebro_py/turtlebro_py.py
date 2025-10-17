import math
import subprocess
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Float32MultiArray, Int16
from std_srvs.srv import Empty
from tf_transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_inverse,
    quaternion_multiply,
)

from turtlebro_speech.srv import Speech

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
        raise TimeoutError(f'No message received on {topic}')
    return future.result()


class TurtleBro:
    """
    Класс для базового робота TurtleBro с управлением движением, светодиодами, камерой и звуком.
    Интерфейс совместим с версией под ROS 1.
    """

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

        self.wait_for_odom_to_start()
        self.sum_target_angle = self.__get_current_angle(self.odom.pose.pose.orientation)

    def __del__(self):
        self.shutdown()

    def shutdown(self):
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

    # Основной метод движения с трапецеидальной скоростью
    def __move(self, meters):
        if DEBUG:
            print('Начало движения на метров:', meters)

        init_x = self.odom.pose.pose.position.x
        init_y = self.odom.pose.pose.position.y
        total_distance = abs(meters)
        direction = 1 if meters > 0 else -1

        vel = Twist()

        while rclpy.ok():
            dx = self.odom.pose.pose.position.x - init_x
            dy = self.odom.pose.pose.position.y - init_y
            distance_passed = math.sqrt(dx**2 + dy**2)

            if distance_passed >= total_distance:
                vel.linear.x = 0.0
                self.vel_pub.publish(vel)
                if DEBUG:
                    print('Движение завершено. Проехано:', round(distance_passed, 2))
                return

            speed = self.__move_trapezoidal_trajectory(
                self.linear_x_val, distance_passed, total_distance
            )
            vel.linear.x = direction * speed
            self.vel_pub.publish(vel)
            time.sleep(0.03)

    # Навигация к точке
    def __goto(self, x, y, theta):
        heading = self.__get_turn_angle_to_point(x, y)
        distance = self.__get_distance_to_point(x, y)
        if DEBUG:
            print('поворот на:', heading)
            print('вперед на:', distance)
        self.__turn(math.degrees(heading))
        self.__move(distance)

    # Метод поворота с трапецеидальной скоростью
    def __turn(self, degrees):
        epsilon = 0.01
        min_speed = 0.05
        total_angle = math.radians(abs(degrees))
        self.sum_target_angle += math.radians(degrees)
        target_angle = self.sum_target_angle % (2 * math.pi)
        turn_dir = 1 if degrees > 0 else -1
        if turn_dir == 0:
            return

        vel = Twist()

        while rclpy.ok():
            current_angle = self.__get_current_angle(self.odom.pose.pose.orientation) % (
                2 * math.pi
            )
            delta = target_angle - current_angle
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi

            if abs(delta) <= epsilon:
                vel.angular.z = 0.0
                self.vel_pub.publish(vel)
                if DEBUG:
                    print(f'Поворот завершен. Угол: {math.degrees(current_angle):.2f}°')
                return

            speed = self.__turn_trapezoidal_trajectory(
                self.angular_z_val, delta, total_angle, min_speed
            )
            vel.angular.z = turn_dir * speed
            self.vel_pub.publish(vel)
            time.sleep(0.05)

    # Вычисление скорости по трапецеидальной траектории для движения
    def __move_trapezoidal_trajectory(self, max_speed, distance_passed, total_distance, min_speed=0.05):
        move_deccel = max(total_distance * 0.5, 0.10)
        distance_remaining = total_distance - distance_passed
        if distance_remaining < move_deccel:
            speed = max(min_speed, max_speed * (distance_remaining / move_deccel))
        else:
            speed = max_speed
        return speed

    # Вычисление скорости по трапецеидальной траектории для поворота
    def __turn_trapezoidal_trajectory(self, max_speed, delta, total_angle, min_speed=0.075):
        turn_deccel = max(total_angle * 0.7, 0.2)
        if abs(delta) < turn_deccel:
            speed = max(min_speed, max_speed * (abs(delta) / turn_deccel))
        else:
            speed = max_speed
        return speed

    # Различные вспомогательные функции для углов и дистанций
    def __get_angle_diff(self, prev_orientation, current_orientation):
        prev_q = [
            prev_orientation.x,
            prev_orientation.y,
            prev_orientation.z,
            prev_orientation.w,
        ]
        current_q = [
            current_orientation.x,
            current_orientation.y,
            current_orientation.z,
            current_orientation.w,
        ]
        delta_q = quaternion_multiply(prev_q, quaternion_inverse(current_q))
        (_, _, yaw) = euler_from_quaternion(delta_q)
        return -yaw

    def __get_current_angle(self, current_orientation):
        current_q = [
            current_orientation.x,
            current_orientation.y,
            current_orientation.z,
            current_orientation.w,
        ]
        (_, _, yaw) = euler_from_quaternion(current_q)
        return yaw

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
    """
    Робот с поддержкой навигации через Nav2 (navigate_to_pose).
    """

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
            raise RuntimeError('NavigateToPose action server not available')

        goal = self.__goal_message_assemble(x, y, theta)
        send_future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('NavigateToPose goal rejected')

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
        assert isinstance(col, str), 'Имя цвета должно быть: red, green, blue, yellow, white или off'
        rgb = {'red': 1, 'green': 2, 'blue': 3, 'yellow': 4, 'white': 5, 'off': 6}
        shade = Int16()
        shade.data = int(rgb[col])
        self.colorpub.publish(shade)

    # Получение расстояния по лазеру
    def distance(self, angle):
        assert isinstance(angle, (int, float)), 'Угол должен быть числом от 0 до 359'
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
                self._node, '/front_camera/image_raw/compressed', CompressedImage, timeout=3.0
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
        assert timeval > 0 and isinstance(timeval, (float, int)), 'Временной интервал должен быть > 0'
        process = subprocess.Popen(
            ['arecord', '-D', 'hw:1,0', '-f', 'S16_LE', '-r 48000', f'/home/pi/{filename}{file_format}']
        )
        time.sleep(timeval)
        process.kill()

    # Произнесение текста
    def say(self, text):
        if not isinstance(text, str):
            text = str(text)
        while not self.speech_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError('Speech service unavailable')
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
