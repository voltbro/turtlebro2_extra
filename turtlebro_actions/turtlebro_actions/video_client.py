#! /usr/bin/env python3
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import CompressedImage


class VideoClient:
    """Blocking helper that fetches a single compressed image frame."""

    def __init__(self, node: Node, topic: str = '/front_camera/compressed') -> None:
        self._node = node
        self._topic = topic
        self._node.get_logger().info('Video client ready on %s', topic)

    def getImage(self, timeout: Optional[float] = None) -> bytes:
        future: Future = Future()

        def _callback(msg: CompressedImage) -> None:
            if not future.done():
                future.set_result(msg.data)

        subscription = self._node.create_subscription(
            CompressedImage,
            self._topic,
            _callback,
            10,
        )
        try:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        finally:
            self._node.destroy_subscription(subscription)

        if future.done():
            return future.result()
        raise TimeoutError(f'No image received on {self._topic}')


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Node('video_client_node')
    try:
        client = VideoClient(node)
        data = client.getImage(timeout=5.0)
        node.get_logger().info('Received compressed image with %d bytes', len(data))
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error('Video client failed: %s', exc)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
