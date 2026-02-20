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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import CompressedImage

from turtlebro_interfaces.srv import Photo

_IMAGE_TOPIC = '/front_camera/image_raw/compressed'


class PhotoService(Node):
    """Сервис, возвращающий последний кадр сжатого изображения камеры."""

    def __init__(self) -> None:
        super().__init__('photo_service')
        self._callback_group = ReentrantCallbackGroup()

        self._service = self.create_service(
            Photo,
            'get_photo',
            self.handle_photo,
            callback_group=self._callback_group,
        )
        self.get_logger().info('Сервис получения фото готов')

    def handle_photo(self, request: Photo.Request, response: Photo.Response) -> Photo.Response:
        success, image = wait_for_message(
            CompressedImage, self, _IMAGE_TOPIC, time_to_wait=5.0,
        )
        if not success:
            self.get_logger().error(
                f'На топик {_IMAGE_TOPIC} не поступило изображение'
            )
            return response
        response.photo = image
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhotoService()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
