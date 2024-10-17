# Implementing a visual servoing algorithm for a 2 DOF robot in ROS2.

This project focuses on implementing visual feedback manipulation on a 2-DOF planar robotic manipulator in Gazebo. The manipulator interacts with an object, modeled as a cube with four circles. The visual servoing algorithm leverages four point features (the centers of the circles) to move the robot between different image configurations.

The robot operates using two controllers:

**Position Controller: ** Ensures that the entire object remains visible within the camera's view, marking the desired configuration.
**Velocity Controller: ** Fine-tunes the robot's movements to perform visual servoing and align the object with the target image configuration.

In the new location, the full object remains visible, allowing seamless manipulation through the virtual camera.

