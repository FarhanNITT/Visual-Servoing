import numpy as np


centers = np.load('/home/farhan/ros2_gazebo/src/opencv_test_py/opencv_test_py/circle_centers_contour.npy',allow_pickle= True)

flattened_points = np.array(centers).flatten()

    # Save the flattened array back to a .npy file
np.save("reference_points.npy", flattened_points)

print(centers)
print(flattened_points)