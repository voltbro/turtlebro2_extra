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
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CompressedImage

from turtlebro_interfaces.srv import Photo


class PhotoService(Node):
    """Сервис, возвращающий последний кадр сжатого изображения камеры."""

    def __init__(self) -> None:
        super().__init__('photo_service')
        self._callback_group = ReentrantCallbackGroup()
        self._last_image: Optional[CompressedImage] = None
        self._image_event = threading.Event()

        self._subscription = self.create_subscription(
            CompressedImage,
            '/front_camera/image_raw/compressed',
            self._image_callback,
            10,
            callback_group=self._callback_group,
        )

        self._service = self.create_service(
            Photo,
            'get_photo',
            self.handle_photo,
            callback_group=self._callback_group,
        )
        self.get_logger().info('Сервис получения фото готов')

    def _image_callback(self, msg: CompressedImage) -> None:
        self._last_image = msg
        self._image_event.set()

    def handle_photo(self, request: Photo.Request, response: Photo.Response) -> Photo.Response:
        timeout_sec = 5.0

        if self._last_image is None:
            if not self._image_event.wait(timeout=timeout_sec):
                self.get_logger().error(
                    'На топик /front_camera/image_raw/compressed не поступило изображение'
                )
                return response

        response.photo = self._last_image
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhotoService()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Сервис фото остановлен пользователем')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
