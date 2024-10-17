#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class VisualServoingNode(Node):
    def __init__(self):
        # Initialize the node
        # rclpy.init_node('visual_servoing_node')
        super().__init__('visual_servoing_node')

        # Parameters for the robot (link lengths)
         # Change based on u  # Change based on urdf


        self.l1 = self.declare_parameter('l1', 1.0)   # change from urdf
        self.l2 = self.declare_parameter('l2', 1.0)    # change from urdf

        # Proportional gain for visual servoing
        self.declare_parameter('lambda_gain', 0.5)  # Customise 

        # # Subscribers
        # rclpy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        # rclpy.Subscriber('/detected_centers', Float64MultiArray, self.feature_points_callback)

        # # Publisher to the predefined controller (publishing joint velocities)
        # self.velocity_pub = rclpy.Publisher('/forward_velocity_controller/commands', Float64MultiArray, queue_size=10)

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)  # 10 is the queue size
        self.create_subscription(Float64MultiArray, '/detected_centers', self.feature_points_callback, 10)

        # Publisher to the predefined controller (publishing joint velocities)
        self.velocity_pub = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)  # 10 is the queue size


        # Internal variables
        self.current_joint_angles = [0, 0]  # [q1, q2]
        self.current_feature_points = None
        self.desired_feature_points = np.load('circle_centers.npy').reshape(4, 2)  # np array
       

        # self.joint_angles = []


    def joint_state_callback(self, msg):
        # Update the current joint angles
        self.current_joint_angles = [msg.position[0], msg.position[1]] * (180/np.pi)   # radians to degrees for FK

    def feature_points_callback(self, msg):
        # Update the current feature points
        if len(msg.data) == 8:
            self.current_feature_points = np.array(msg.data).reshape(4, 2)
            self.visual_servoing()
        else:
            self.get_logger().warn("Incorrect number of feature points received.")

    def compute_image_jacobian(self, u, v):
        # Compute the image Jacobian for one feature point
        return np.array([
            [-1, 0, u, u*v, -(1 + u**2), v],
            [0, -1, v, 1 + v**2, -u*v, -u]
        ])

    def compute_robot_jacobian(self, q1, q2):
        # Compute the Jacobian for the planar 2R robot
        return np.array([
            [-self.l1 * np.sin(q1) - self.l2 * np.sin(q1 + q2), -self.l2 * np.sin(q1 + q2)],
            [self.l1 * np.cos(q1) + self.l2 * np.cos(q1 + q2), self.l2 * np.cos(q1 + q2)]
        ])

    def visual_servoing(self):
        if self.current_feature_points is None:
            return  # Don't proceed if no feature points have been received

        # Compute the error between current and desired feature points
        errors = [self.current_feature_points[i] - self.desired_feature_points[i] for i in range(4)]

        # Compute the total image Jacobian
        J_total = np.vstack([self.compute_image_jacobian(u, v) for (u, v) in self.current_feature_points])

        # Stack the errors into a single vector
        e_total = np.hstack(errors)

        # Compute the pseudo-inverse of the total image Jacobian
        J_pseudo_inv = np.linalg.pinv(J_total)

        # Compute end-effector velocity
        v_c = -self.lambda_gain * J_pseudo_inv.dot(e_total)

        # Extract the current joint angles
        q1, q2 = self.current_joint_angles

        # Compute the robot's Jacobian
        J_robot = self.compute_robot_jacobian(q1, q2)

        # Compute joint velocities
        joint_velocities = np.linalg.inv(J_robot).dot(v_c)

        # Publish joint velocities to the controller
        velocity_msg = Float64MultiArray()
        velocity_msg.data = joint_velocities 
        self.velocity_pub.publish(velocity_msg)
        self.get_logger().info(f"Published joint velocities: {joint_velocities}")  # Use get_logger() in ROS 2


if __name__ == '__main__':

      # Initialize the rclpy library
    rclpy.init(args=None)

    # Create the node
    visual_servo = VisualServoingNode()
    
    # Spin the node so the callback function is called.
    rclpy.spin(visual_servo)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    visual_servo.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
