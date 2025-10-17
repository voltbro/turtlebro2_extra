#! /usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import CompressedImage

from turtlebro_actions.srv import Photo


class PhotoService(Node):
    """ROS 2 service that returns the latest compressed camera frame."""

    def __init__(self) -> None:
        super().__init__('photo_service')
        self._service = self.create_service(Photo, 'get_photo', self.handle_photo)
        self.get_logger().info('Photo service ready')

    def handle_photo(self, request: Photo.Request, response: Photo.Response) -> Photo.Response:
        try:
            response.photo = self._wait_for_photo('/front_camera/image_raw/compressed', timeout=5.0)
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
            raise TimeoutError(f'No photo received on {topic}')
        return future.result()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhotoService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Photo service interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
