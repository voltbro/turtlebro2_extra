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
from copy import deepcopy
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from tf_transformations import euler_from_quaternion, quaternion_multiply

from turtlebro_actions.action import Rotation


class RotateServer(Node):
    """ROS 2 action server that rotates the robot around the Z axis."""

    def __init__(self) -> None:
        super().__init__('rotate_server_node')
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = Odometry()
        self.create_subscription(Odometry, '/odom', self.subscriber_odometry_cb, 10)

        self._action_server = ActionServer(
            self,
            Rotation,
            'action_rotate',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info('Start rotation ActionServer')

    def subscriber_odometry_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def goal_callback(self, goal_request: Rotation.Goal) -> GoalResponse:
        self.get_logger().info(
            'Received rotate goal degrees=%d speed=%.3f',
            goal_request.goal,
            goal_request.speed,
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Received cancel request for rotate goal')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        prev_orientation = deepcopy(self.odom.pose.pose.orientation)
        accumulated_degrees = 0.0

        cmd = Twist()
        cmd.angular.z = -abs(goal.speed) if goal.goal > 0 else abs(goal.speed)
        self.cmd_vel.publish(cmd)

        feedback = Rotation.Feedback()
        rate = self.create_rate(20)

        self.get_logger().info('Executing rotate goal')

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Rotate goal canceled')
                self.cmd_vel.publish(Twist())
                goal_handle.canceled()
                result = Rotation.Result()
                result.result = int(round(accumulated_degrees))
                return result

            current_orientation = self.odom.pose.pose.orientation
            delta = self._get_degree_diff(prev_orientation, current_orientation)
            accumulated_degrees += delta
            prev_orientation = deepcopy(current_orientation)

            feedback.feedback = int(round(accumulated_degrees))
            goal_handle.publish_feedback(feedback)

            if self._is_goal_reached(accumulated_degrees, goal.goal):
                self.cmd_vel.publish(Twist())
                goal_handle.succeed()
                result = Rotation.Result()
                result.result = int(round(accumulated_degrees))
                self.get_logger().info('Rotate goal succeeded')
                return result

            await rate.sleep()

        self.cmd_vel.publish(Twist())
        goal_handle.abort()
        result = Rotation.Result()
        result.result = int(round(accumulated_degrees))
        return result

    @staticmethod
    def _get_degree_diff(prev_orientation, current_orientation) -> float:
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
        # quaternion_multiply returns the composition prev * current
        delta_q = quaternion_multiply(prev_q, current_q)
        _, _, yaw = euler_from_quaternion(delta_q)
        return math.degrees(yaw)

    @staticmethod
    def _is_goal_reached(current_degrees: float, goal_degrees: int) -> bool:
        if goal_degrees >= 0:
            return current_degrees >= goal_degrees
        return current_degrees <= goal_degrees


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RotateServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Rotate server interrupted by user')
    finally:
        node.cmd_vel.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
