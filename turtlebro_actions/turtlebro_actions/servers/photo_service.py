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
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import CompressedImage

from turtlebro_interfaces.srv import Photo


class PhotoService(Node):
    """Сервис, возвращающий последний кадр сжатого изображения камеры."""

    def __init__(self) -> None:
        super().__init__('photo_service')
        self._service = self.create_service(Photo, 'get_photo', self.handle_photo)
        self.get_logger().info('Сервис получения фото готов')

    def handle_photo(self, request: Photo.Request, response: Photo.Response) -> Photo.Response:
        try:
            response.photo = self._wait_for_photo(
                '/front_camera/image_raw/compressed', timeout=5.0
            )
        except TimeoutError as exc:
            self.get_logger().error(str(exc))
        return response

    def _wait_for_photo(self, topic: str, timeout: Optional[float]) -> CompressedImage:
        future: Future = Future()

        def _callback(msg: CompressedImage) -> None:
            if not future.done():
                future.set_result(msg)

        subscription = self.create_subscription(CompressedImage, topic, _callback, 10)
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        finally:
            self.destroy_subscription(subscription)

        if not future.done():
            raise TimeoutError(f'На топик {topic} не поступило изображение')
        return future.result()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhotoService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Сервис фото остановлен пользователем')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
