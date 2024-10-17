import numpy as np
import cv2



def getJinverse():  # confirm this once 
    # J= np.array([   [0,0],
    #                 [0,0],
    #                 [1,0.5],
    #                 [-1,-1],
    #                 [0,0],
    #                 [0,0]])
    J= np.array([   [-1,-0.5],
                    [0,0],
                    [0,0],
                    [0,0],
                    [0,0],
                    [-1,-1]])
    return np.linalg.pinv(J)



def singlept_img_jacob(f, Z, x, y):
    jacob = np.array([  [-f/Z, 0, x/Z, x*y/f, -1*(f+x*x/f), y ],
                    [0, -f/Z, y/Z, f+y*y/f, -x*y/f, -x]])

    # return np.linalg.pinv(Lx)
    return jacob

def img_jacoabian_inv(f,Z, current_pt):
    jaco=singlept_img_jacob(f,Z,current_pt[0], current_pt[1])
    i=2

    while(i<len(current_pt)):
       jaco=np.concatenate((jaco, singlept_img_jacob(f, Z, current_pt[i], current_pt[i+1])), axis=0)
       i=i+2
    
    condition_number = np.linalg.cond(jaco)

    if condition_number > 1e5:  # Threshold depends on your tolerance
        print(f"Jacobian is ill-conditioned: {condition_number}")
        return np.zeros_like(jaco)  # Handle singularity by returning zero velocity
    
    return np.linalg.pinv(jaco)

def centroid(coordinates):
    centroids = []
    
    # Process the coordinates in chunks of 8 (representing 4 (x, y) pairs)
    for i in range(0, len(coordinates), 8):
        # Extract 4 pairs of (x, y)
        xy_pairs = np.array(coordinates[i:i+8])
        
        xy_pairs = np.reshape(xy_pairs,(-1, 2))
        
        # Compute the centroid of the 4 points
        centroid = np.mean(xy_pairs, axis=0)
        centroids.append(centroid)

    return np.array(centroids).flatten()

def camera_vel_twist(f,Z, flambda, current_feature_pt):

    # feature_pt = centroid(current_feature_pt)
    # error = feature_error(feature_pt) 
    error = feature_error(current_feature_pt) 
    print("Error- ", error)

    if np.linalg.norm(error) < 1e-3:  # Example threshold
        print("Error below threshold, stopping robot.")
        return np.zeros(6)  # Return zero velocity
    
    img_jacobian_inverse= img_jacoabian_inv(f, Z, current_feature_pt) 
    # img_jacobian_inverse= img_jacoabian_inv(f, Z, feature_pt) 
    camera_twist = -flambda * np.matmul(img_jacobian_inverse, error)
    return camera_twist

def Image2Pixel(x,y):
    transformation_matrix= np.linalg.inv(IntrinsicCameraMatrix())
    img= np.array([x,y,1])
    return np.matmul(transformation_matrix, img)

def IntrinsicCameraMatrix():
    mat= np.array([ [476.7030836014194, 0.0, 400.0],
                    [0.0, 476.7030836014194, 400.0],
                    [0.0, 0.0, 1.0]])
    return mat

def feature_error(current_pt):
    error=[]

    i=0
    while(i<len(current_pt)):
        point_now_p=Image2Pixel(current_pt[i], current_pt[i+1])
        point_ref_p=Image2Pixel(point_ref[i], point_ref[i+1]) 
        error.append(point_ref_p[0]- point_now_p[0])
        error.append(point_ref_p[1]- point_now_p[1])
        i=i+2

    return np.array(error)

def LoadPointsFromNPY():
    points = np.load('/home/sarthak_m/VBRM/LabSession/ros2_ws_gazebo/src/opencv_test_py/opencv_test_py/centroid_desired.npy')
    return points.tolist()  # Convert the NumPy array to a Python list

def circlecenter_points(img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # converted to HSV color space -- better for varying lighting conditions and simulation
        
        blue_lower_bound = np.array([120,100, 130])
        blue_upper_bound = np.array([120,255, 255])
        blue_mask= cv2.inRange(hsv, blue_lower_bound, blue_upper_bound)
        

        green_lower_bound = np.array([59,220, 130])
        green_upper_bound = np.array([61,255, 255])
        green_mask= cv2.inRange(hsv, green_lower_bound, green_upper_bound)
        
        red_lower_bound = np.array([0,100, 130])
        red_upper_bound = np.array([5,255, 255])
        red_mask= cv2.inRange(hsv, red_lower_bound, red_upper_bound)

        pink_lower_bound = np.array([150,220, 130])
        pink_upper_bound = np.array([150,255, 255])
        pink_mask= cv2.inRange(hsv, pink_lower_bound, pink_upper_bound)

        circle_names= ['blue_circle', 'green_circle', 'red_circle', 'pink_circle']
        circles=[blue_mask, green_mask, red_mask, pink_mask]

        num=0
        centers= []
        count = 1.0
        for circle in circles:
            y_indices, x_indices = np.where(circle == 255)

            if len(x_indices) > 0 and len(y_indices) > 0: 
                cX = np.mean(x_indices)   # Calculate the center as the mean of all non-zero pixel coordinates
                cY = np.mean(y_indices)

                centers.append(cX)  # Add x-coordinate to the list
                centers.append(cY)  # Add y-coordinate to the list
                centers.append(count)
                print(f"{circle_names[num]} center -> ({cX}, {cY})")

            else :

                print(f"No points detected for {circle_names[num]}")
                # centers.append(None)  # If no center is found, append None
                # centers.append(None)  # Append None for both x and y

                # centers.append(-1.0)  # If no center is found, append a default value (e.g., -1.0)
                # centers.append(-1.0) 

                
            num += 1
            count +=1

        return centers


point_ref = LoadPointsFromNPY()





