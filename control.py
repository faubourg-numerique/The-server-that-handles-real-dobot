#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64, String
from math import atan2, degrees, radians, sqrt, acos

class ArmActionClient(Node):

    def __init__(self):
        super().__init__('arm_control_actionclient')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        # Initialize subscribers for target position and rotation
        self.create_subscription(Float64, '/target_r', self.r_callback, 10)
        self.create_subscription(Float64, '/target_x', self.x_callback, 10)
        self.create_subscription(Float64, '/target_y', self.y_callback, 10)
        self.create_subscription(Float64, '/target_z', self.z_callback, 10)

        # Initialize variables to store joint angles and target coordinates
        self.ax1 = 0.0
        self.ax2 = 0.0
        self.ax3 = 0.0
        self.ax4 = 0.0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.r = 0.0

    def x_callback(self, msg):
        self.x = msg.data

    def y_callback(self, msg):
        self.y = msg.data

    def z_callback(self, msg):
        self.z = msg.data
        self.calculate_and_send_goal()

    def r_callback(self, msg):
        self.r = msg.data

    def calculate_and_send_goal(self):
        success = self.calc_inv_kin()
        if success:
            self.send_goal()
        else:
            self.get_logger().error('Failed to calculate inverse kinematics')

    def calc_inv_kin(self):
        try:
            rear_arm = 135
            forearm = 147

            self.ax4 = radians(self.r)  # Wrist rotation angle

            self.ax1 = atan2(self.y, self.x)  # Base rotation angle

            hypotenuse = self.x**2 + self.y**2 + self.z**2
            beta = atan2(self.z, sqrt(self.x**2 + self.y**2))
            psi = abs(acos((hypotenuse + rear_arm**2 - forearm**2) / (2 * rear_arm * sqrt(hypotenuse))))

            if beta >= 0:
                self.ax2 = radians(90) - (beta + psi)
            else:
                self.ax2 = radians(90) - (psi - abs(beta))

            theta_3 = abs(acos((hypotenuse - rear_arm**2 - forearm**2) / (2 * rear_arm * forearm)))
            self.ax3 = theta_3 - radians(90) + self.ax2

            return True
        except Exception as e:
            self.get_logger().error(f'Error in inverse kinematics: {e}')
            return False

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        joint_names = ["ax1_joint", "ax2_joint", "ax3_joint", "ax4_joint"]

        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=10).to_msg()
        point.positions = [self.ax1, self.ax2, self.ax3, self.ax4]

        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = [point]
        goal_msg.goal_time_tolerance = Duration(seconds=1).to_msg()

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')


def main(args=None):
    rclpy.init(args=args)
    action_client = ArmActionClient()
    rclpy.spin(action_client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
