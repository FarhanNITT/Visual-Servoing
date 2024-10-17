import numpy as np
import matplotlib.pyplot as plt

def read_trajectory(file_path):
    blue_x, blue_y = [], []
    green_x, green_y = [], []
    red_x, red_y = [], []
    pink_x, pink_y = [], []

    with open(file_path, 'r') as f:
        lines = f.readlines()
        # Filter out empty lines and strip whitespace
        filtered_lines = [line.strip() for line in lines if line.strip()]
        
        # Read 12 lines at a time
        for i in range(0, len(filtered_lines), 12):
            if i + 11 < len(filtered_lines):  # Ensure we have enough lines left
                # Parse the 12 lines into float values
                values = np.array([float(filtered_lines[j]) for j in range(i, i + 12)])
                
                # Append the values to the corresponding lists
                blue_x.append(values[0])
                blue_y.append(values[1])
                green_x.append(values[3])
                green_y.append(values[4])
                red_x.append(values[6])
                red_y.append(values[7])
                pink_x.append(values[9])
                pink_y.append(values[10])

    return blue_x, blue_y, green_x, green_y, red_x, red_y, pink_x, pink_y

def plot_trajectories(file_path):
    blue_x, blue_y, green_x, green_y, red_x, red_y, pink_x, pink_y = read_trajectory(file_path)
    
    # Debugging outputs to check data
    print(f"Blue Circle: {len(blue_x)} points")
    print(f"Green Circle: {len(green_x)} points")
    print(f"Red Circle: {len(red_x)} points")
    print(f"Pink Circle: {len(pink_x)} points")
    
    plt.figure(figsize=(10, 8))
    
    # Plot each trajectory
    if blue_x and blue_y:
        plt.plot(blue_x, blue_y, label='Blue Circle', color='blue')
        plt.scatter(blue_x[0], blue_y[0], marker='o', color='blue', s=100, label='Blue Start')  # Start point
        plt.scatter(blue_x[-1], blue_y[-1], marker='s', color='blue', s=100, label='Blue End')  # End point
    if green_x and green_y:
        plt.plot(green_x, green_y, label='Green Circle', color='green')
        plt.scatter(green_x[0], green_y[0], marker='o', color='green', s=100, label='Green Start')  # Start point
        plt.scatter(green_x[-1], green_y[-1], marker='s', color='green', s=100, label='Green End')  # End point
    if red_x and red_y:
        plt.plot(red_x, red_y, label='Red Circle', color='red')
        plt.scatter(red_x[0], red_y[0], marker='o', color='red', s=100, label='Red Start')  # Start point
        plt.scatter(red_x[-1], red_y[-1], marker='s', color='red', s=100, label='Red End')  # End point
    if pink_x and pink_y:
        plt.plot(pink_x, pink_y, label='Pink Circle', color='magenta')
        plt.scatter(pink_x[0], pink_y[0], marker='o', color='magenta', s=100, label='Pink Start')  # Start point
        plt.scatter(pink_x[-1], pink_y[-1], marker='s', color='magenta', s=100, label='Pink End')  # End point

    # Reference points directly as coordinates
    reference_points = {
        'Blue Ref': (428, 288),
        'Green Ref': (510, 372),
        'Red Ref': (428, 370),
        'Pink Ref': (512, 286)
    }

    # Plot reference points as crosses
    for label, (x, y) in reference_points.items():
        plt.scatter(x, y, label=label, marker='x', color='black', s=100)  # You can change the color or size as needed

    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Trajectory of Circles and Reference Points')
    plt.legend()
    plt.grid(True)
    plt.show()

# Path to your trajectory text file
file_path = '/home/sarthak_m/VBRM/LabSession/ros2_ws_gazebo/src/opencv_test_py/opencv_test_py/trajectory_1.txt'

# Plot the trajectories
plot_trajectories(file_path)
