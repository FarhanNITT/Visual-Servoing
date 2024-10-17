# Implementing a visual servoing algorithm for a 2 DOF robot in ROS2.

This project focuses on implementing visual feedback manipulation on a 2-DOF planar robotic manipulator in Gazebo. The manipulator interacts with an object, modeled as a cube with four circles. The visual servoing algorithm leverages four point features (the centers of the circles) to move the robot between different image configurations.

Circle centers are detected using two methods:

1) Mean Color Thresholding
2) Hough Circle Transforms

The robot operates using two controllers:

**Position Controller:**  Ensures that the entire object remains visible within the camera's view, marking the desired configuration.

**Velocity Controller:**  Fine-tunes the robot's movements to perform visual servoing and align the object with the target image configuration.

In the new location, the full object remains visible, allowing seamless manipulation through the virtual camera.

# Overview 

The robot is first moved to the desired configuration using the position controller, where the circle center detection algorithm is executed. The detected centers are then saved as reference points for the Image-Based Visual Servoing (IBVS) control loop. Next, the robot is moved to the initial configuration. The goal is to transition the robot from this initial state to the predefined desired configuration by tracking the detected feature points' trajectories through visual feedback. The IBVS algorithm is run with 𝑓 = 𝑍 = 1 to compute the required joint velocities, and the robot is then moved using the velocity controller based on these calculations.

<div style="display: flex; justify-content: center; gap: 100px;">
  <img src="src/Initial_config_centers.jpg" alt="First Image" width="470" style="margin: 0 10px;" />
  <img src="src/desired_config_centers.jpg" alt="Second Image" width="470" style="margin: 0 10px;" />
</div>

<hr style="border: 1px solid #ccc; margin: 20px 0;">

<div style="text-align: center;">
  <strong>The control loop utilized in this project is based on</strong>
</div>



![Alt text for third image](src/IBVS1.png)


The robot trajectory obtained through the visual servoing algorithm effectively tracked the centers of the circles, ensuring precise movement from the initial configuration to the desired position while maintaining visual feedback throughout the process.

![Alt text for third image](src/trajectory_plot.png)


# Instructions to run the repository:

### 1) Create a new workspace and navigate to your workspace's directory:

      cd ~/your_workspace/

###  2) Clone the repository:

      git clone https://github.com/FarhanNITT/Visual-Servoing.git

 ### 3) Build the workspace:

      colcon build --symlink install
      
 ### 4) Source the workspace:

      source install/setup.bash
      
 ### 5) Run the simulation and spwan the robot:

     ros2 launch rrbot_gazebo rrbot_world.launch.py
      
###  6) Run the simulation and spwan the robot:

      ros2 launch rrbot_gazebo object_spawn.launch.py

###   7) Move the robot to an initial position.
     
      ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray
      "{data: [0.25, -1.0]}"

###   8) Start the velocity controller
     
      ros2 control switch_controllers --start forward_velocity_controller --stop
      forward_position_controller

###   9) Start the velocity controller
     
      ros2 control switch_controllers --start forward_velocity_controller --stop
      forward_position_controller
  
###   10) Open a new terminal and navigate to :
      cd src/opencv_test_py/opencv_test_py
      python3 feature_points.py

###   11) Open a new terminal and navigate to :
      cd src/opencv_test_py/opencv_test_py
      python3 visual_servoing.py
