import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np

from . import helper_functions


class VisualServoingNode(Node):
    def __init__(self):
        
        super().__init__('visual_servoing_node')

        self.velocity_pub = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)  # 10 is the queue size
        self.subscriber_= self.create_subscription(Twist, 
                                'camera_velocity', self.listener_callback, 10)
        

    def listener_callback(self,msg):

        camera_twist= np.array([[msg.linear.x], [msg.linear.y], [msg.linear.z],
                            [msg.angular.x], [msg.angular.y], [msg.angular.z]])
        robot_jacob_inv= helper_functions.getJinverse()

        joint_vel_values= np.matmul(robot_jacob_inv, camera_twist)
        joint_vel_values[(joint_vel_values<0.002) & (joint_vel_values>-0.002)]=0

        msg=Float64MultiArray()
        msg.data= [float(joint_vel_values[0]), float(joint_vel_values[1])]
        self.velocity_pub.publish(msg)
        print('joint velocity- ', joint_vel_values)


def main(args=None):
    rclpy.init(args=args)

    joint_velocity_publisher= VisualServoingNode()
    rclpy.spin(joint_velocity_publisher)

    joint_velocity_publisher.destroy_node()
    rclpy.shutdown

if __name__=='__main__':
    main()
        



