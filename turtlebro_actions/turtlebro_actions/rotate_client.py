#! /usr/bin/env python3
import sys
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from turtlebro_actions.action import Rotation


class RotateClient:
    """ROS 2 action client wrapper that mirrors the previous ROS 1 interface."""

    def __init__(self, node: Node, wait_timeout: float = 5.0) -> None:
        self._node = node
        self._client = ActionClient(node, Rotation, 'action_rotate')
        if not self._client.wait_for_server(timeout_sec=wait_timeout):
            raise RuntimeError('Rotation action server not available')
        self._node.get_logger().info('Rotation action client connected')

    def SendGoal(self, goal_val: int, speed_val: float) -> Rotation.Result:
        goal = Rotation.Goal()
        goal.goal = int(goal_val)
        goal.speed = float(speed_val)

        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._action_feedback,
        )
        rclpy.spin_until_future_complete(self._node, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self._node.get_logger().warn('Rotation goal rejected')
            result = Rotation.Result()
            result.result = 0
            return result

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        goal_result = result_future.result()

        if goal_result is None:
            result = Rotation.Result()
            result.result = 0
            return result

        return goal_result.result

    def _action_feedback(self, feedback: Rotation.Feedback) -> None:
        self._node.get_logger().info('Current angle: %d', feedback.feedback)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Node('rotate_client_node')
    try:
        client = RotateClient(node)
        result = client.SendGoal(10, 1.2)
        node.get_logger().info('Action Result angle: %d', result.result)
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error('Rotate client failed: %s', exc)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
