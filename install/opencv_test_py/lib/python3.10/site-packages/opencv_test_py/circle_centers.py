# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# This code is modified by Berk Calli from the following author.
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

import cv2 # OpenCV library
import numpy as np
from std_msgs.msg import Float64MultiArray 

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.

    self.subscription = self.create_subscription(
      Image, 
      '/camera1/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'output_image', 10)

    self.center_pub = self.create_publisher(Float64MultiArray,'detected_center',10)  # create a publisher to publish detected centers 

      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

    current_frame_gray = cv2.GaussianBlur(current_frame_gray, (7, 7), 2) 

    circles = cv2.HoughCircles(current_frame_gray, 
                            cv2.HOUGH_GRADIENT, 
                            dp=2,   # ratio of accumulator circle array and input image resolution
                            minDist=20,   # minimum distance between 2 detected circle to avoid detecting multiple circles for same object
                            param1=75,   # upper threshold for canny edge 
                            param2=45,    # threshold to filted detected centers
                            minRadius=10,   # min and max radium of circles to be detected
                            maxRadius=35)
    

    # we define the message datatype to initialize
    center_msg = Float64MultiArray() 

    if circles is not None:
      circles = np.uint16(np.around(circles))
      for i in circles[0, :]:

        center_x, center_y = i[0], i[1]   # extract centers
        self.get_logger().info(f'Detected circle center: ({center_x}, {center_y})')   # Log information 
        # Draw the outer circle
        cv2.circle(current_frame, (i[0], i[1]), i[2], (0, 0, 0), 2)
       
        # Draw the center of the circle
        cv2.circle(current_frame, (i[0], i[1]), 2, (0, 0, 255), 3)

        center_msg.data.extend([center_x, center_y])   # write the message 

        # Optionally, you can also write the coordinates on the frame
        cv2.putText(current_frame, f'({center_x}, {center_y})', 
                    (center_x, center_y - 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, (255, 255, 255), 1, cv2.LINE_AA)

    self.center_pub.publish(center_msg)


    # current_frame_gray = np.float32(current_frame_gray)
    # cannyedge = cv2.Canny(current_frame_gray,100,200)  

    # corners = cv2.cornerHarris(current_frame_gray,2,3,0.04)

    # corners = cv2.dilate(corners,None)

    # th = 0.02 * corners.max()

    # current_frame[corners > th] = [0,0,255]

    # PLACE YOUR CODE HERE. PROCESS THE CURRENT FRAME AND PUBLISH IT. IF YOU ARE HAVING DIFFICULTY PUBLISHING IT YOU CAN USE THE FOLLOWING LINES TO DISPLAY IT VIA OPENCV FUNCTIONS
  
    cv2.imshow("output_image", current_frame)
    # cv2.imwrite('hough_circle_transform_centers.jpg',current_frame)
    cv2.waitKey(1)
    

    # Publish the image.
    # The 'cv2_to_imgmsg' method converts an OpenCV
    # image to a ROS 2 image message
    self.publisher_.publish(self.br.cv2_to_imgmsg(current_frame, encoding="bgr8"))

    
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
