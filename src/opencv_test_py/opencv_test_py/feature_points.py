import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from std_msgs.msg import Float64MultiArray 
import cv2 # OpenCV library
import numpy as np
# from std_msgs.msg import Float64MultiArray
from helper_functions import *

class FeaturePoints(Node):

    def __init__(self):

        super().__init__('feature_points')

        self.subscription = self.create_subscription(Image, '/camera1/image_raw', self.listener_callback, 10)
        self.subscription # prevent unused variable warning
        self.publisher_ = self.create_publisher(Image, 'output_image', 10)  # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.

        self.center_pub = self.create_publisher(Float64MultiArray,'detected_center',10)  # create a publisher to publish detected centers 
        self.br = CvBridge()    # Used to convert between ROS and OpenCV images

    def listener_callback(self,data):
        

        self.get_logger().info('Receiving video frame')     # Display the message on the console

        current_frame = self.br.imgmsg_to_cv2(data)  # ros message to cv2 data type 
        center_points= circlecenter_points(current_frame)    # current image frame as input, we return the circle centers 

        center_msg= Float64MultiArray()
        print(type(center_msg.data))
        center_msg.data = center_points
        self.center_pub.publish(center_msg)    # publish the circle centers on the topic 'detected_center' defined earlier

        # self.publisher_.publish(self.br.cv2_to_imgmsg(current_frame, encoding="bgr8"))
        print("publishing- ", center_msg.data)


  
def main(args=None):
  
  rclpy.init(args=args) # Initialize the rclpy library
  image_subscriber = FeaturePoints()  # Create the node
  rclpy.spin(image_subscriber)   # Spin the node so the callback function is called.
  image_subscriber.destroy_node()     # Destroy the node explicitly
  
  rclpy.shutdown()  # Shutdown the ROS client library for Python
  
if __name__ == '__main__':
  main()



