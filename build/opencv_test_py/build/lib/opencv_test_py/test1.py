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

      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    # self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    #current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

    # thresholding blue color
    # self.get_logger().info(f'Frame shape: {current_frame.shape}')

    # lower_color_bounds = cv2.cv.Scalar(100, 0, 0)
    # upper_color_bounds = cv2.cv.Scalar(225,80,80)

    blue_lower_bound = np.array([100, 0, 0], dtype=np.uint8)  # BGR lower bound
    blue_upper_bound = np.array([225, 80, 80], dtype=np.uint8)  

    red_lower_bound = np.array([0, 0, 100], dtype=np.uint8)  # BGR lower bound
    red_upper_bound = np.array([80, 80, 255], dtype=np.uint8) 

    hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

    green_lower_bound = np.array([35, 100, 100], dtype=np.uint8)  # BGR lower bound
    green_upper_bound = np.array([85, 255, 255], dtype=np.uint8) 

    pink_lower_bound = np.array([140, 50, 50], dtype=np.uint8)  # BGR lower bound
    pink_upper_bound = np.array([170, 255, 255], dtype=np.uint8) 
    
    #print("Current frame dtype:", current_frame.dtype)
    # mask = cv2.inRange(current_frame,lower_color_bounds,upper_color_bounds)
    mask_blue = cv2.inRange(current_frame,blue_lower_bound,blue_upper_bound) 
    mask_red = cv2.inRange(current_frame,red_lower_bound,red_upper_bound) 
    mask_green = cv2.inRange(hsv_frame,green_lower_bound,green_upper_bound) 
    mask_pink = cv2.inRange(hsv_frame,pink_lower_bound,pink_upper_bound)    


    centers = {}        # We will save each color centre and its name in a dictionary and print
    def find_center(mask, color_name):
        # Get coordinates of all non-zero pixels in the mask
        y_indices, x_indices = np.where(mask > 0)
        if len(x_indices) > 0 and len(y_indices) > 0:
            cX = int(np.mean(x_indices))
            cY = int(np.mean(y_indices))

            centers[color_name]=(cX,cY)
            # Draw the center on the original image
            cv2.circle(current_frame, (cX, cY), 5, (255, 255, 255), -1)
            cv2.putText(current_frame, f"{color_name} center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        else :
            centers[color_name] = None  # No center found

    
    combined_mask = cv2.bitwise_or(mask_blue, mask_red)
    combined_mask = cv2.bitwise_or(combined_mask, mask_green)
    combined_mask = cv2.bitwise_or(combined_mask, mask_pink)

    find_center(mask_blue,'blue')
    find_center(mask_red,'red')
    find_center(mask_green,'green')
    find_center(mask_pink,'pink')

    for color, center in centers.items():
        if center:
            self.get_logger().info(f"Center of {color} found at: {center}")
        else:
            self.get_logger().info(f"Center of {color} not found")

    # PLACE YOUR CODE HERE. PROCESS THE CURRENT FRAME AND PUBLISH IT. IF YOU ARE HAVING DIFFICULTY PUBLISHING IT YOU CAN USE THE FOLLOWING LINES TO DISPLAY IT VIA OPENCV FUNCTIONS
    #cv2.imshow("output_image_b", mask_blue)
    cv2.imshow("output_image_centers", current_frame)
    # cv2.imshow("output_image_r", mask_red)
    # cv2.imshow("output_image_g", mask_green)
    #cv2.imshow("output_image_combined", combined_mask)
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
