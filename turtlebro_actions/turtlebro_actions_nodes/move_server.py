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

from turtlebro_actions.action import Move


class MoveServer(Node):
    """ROS 2 action server that drives the robot forward or backward."""

    def __init__(self) -> None:
        super().__init__('move_server_node')
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = Odometry()
        self.create_subscription(Odometry, '/odom', self.subscriber_odometry_cb, 10)

        self._action_server = ActionServer(
            self,
            Move,
            'action_move',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info('Start move ActionServer')

    def subscriber_odometry_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def goal_callback(self, goal_request: Move.Goal) -> GoalResponse:
        self.get_logger().info(
            'Received move goal distance=%.3f speed=%.3f',
            goal_request.goal,
            goal_request.speed,
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Received cancel request for move goal')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        start_pose = deepcopy(self.odom.pose.pose.position)
        target_distance = goal.goal
        direction = 1.0 if target_distance >= 0.0 else -1.0
        commanded_speed = abs(goal.speed) * direction

        cmd = Twist()
        cmd.linear.x = commanded_speed
        self.cmd_vel.publish(cmd)

        feedback = Move.Feedback()
        rate = self.create_rate(20)

        self.get_logger().info('Executing move goal')

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Move goal canceled')
                self.cmd_vel.publish(Twist())
                goal_handle.canceled()
                result = Move.Result()
                result.result = float(
                    self._compute_distance(start_pose, self.odom.pose.pose.position)
                )
                return result

            distance = self._compute_distance(
                start_pose, self.odom.pose.pose.position
            )
            feedback.feedback = float(distance)
            goal_handle.publish_feedback(feedback)

            if self._is_goal_reached(distance, target_distance):
                self.cmd_vel.publish(Twist())
                goal_handle.succeed()
                result = Move.Result()
                result.result = float(distance)
                self.get_logger().info('Move goal succeeded')
                return result

            await rate.sleep()

        # Fallback in case rclpy shuts down
        self.cmd_vel.publish(Twist())
        goal_handle.abort()
        result = Move.Result()
        result.result = 0.0
        return result

    @staticmethod
    def _compute_distance(start_pose, current_pose) -> float:
        dx = start_pose.x - current_pose.x
        dy = start_pose.y - current_pose.y
        return math.sqrt(dx * dx + dy * dy)

    @staticmethod
    def _is_goal_reached(distance: float, goal_value: float) -> bool:
        return distance >= abs(goal_value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Move server interrupted by user')
    finally:
        node.cmd_vel.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
