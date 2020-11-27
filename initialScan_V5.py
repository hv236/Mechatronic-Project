"""
Code to...

@Author: Jerome De Saulles

"""

# This is the vision library OpenCV
import cv2
# This is a library for mathematical functions for python (used later)
import numpy as np
# This is a library to get access to time-related functionalities
import time
# Import the AruCo library
import cv2.aruco as aruco
# Import the math library
import math
import itertools as it

import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')

def rotationMatrixToEulerAngles(R) :

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

def second_smallest(numbers):
    m1, m2 = float('inf'), float('inf')
    for x in numbers:
        if x <= m1:
            m1, m2 = x, m1
        elif x < m2:
            m2 = x
    return m2

def vision():

    
    # Load the camera calibration values
    Camera = np.load('Calibration_Surface_Book_Rear_Camera.npz')
    CM = Camera['CM']  # camera matrix
    dist_coef = Camera['dist_coef']  # distortion coefficients from the camera

    aruco_dict = aruco.Dictionary_get(
        aruco.DICT_4X4_50)  # Load the aruco dictionary
    pa = aruco.DetectorParameters_create()  # Set the detection parameters

    # Select the correct camera (0) = front camera, (1) = rear camera
    cap = cv2.VideoCapture(1)

    # Set the width and heigth of the camera to 640x480
    cap.set(3, 640)
    cap.set(4, 480)

    # Create two opencv named windows
    cv2.namedWindow("frame-image", cv2.WINDOW_AUTOSIZE)
    # cv2.namedWindow("Trackbars")

    # Position the window
    cv2.moveWindow("frame-image", 0, 0)

    # Create Coordinate Storage Arrays
    X = [] #X Coordinate Locations Array
    Y = []
    Z = []
    ID = []
    RVECS = np.array([[0, 0, 0]])
    # RVECS = [[[]]]

    t_end = time.time() + 1000

    # Execute this continuously
    while time.time() < t_end:
        # Capture current frame from the camera
        ret, frame = cap.read()

        # Convert the BGR image to HSV image.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Get the new values of the trackbar in real time as the user changes 
        # them
        l_h = 45
        l_s = 60
        l_v = 60
        u_h = 179
        u_s = 255
        u_v = 255

        # Set the lower and upper HSV range according to the value selected
        # by the trackbar
        lower_range = np.array([l_h, l_s, l_v])
        upper_range = np.array([u_h, u_s, u_v])

        # Filter the image and get the binary mask, where white represents 
        # your target color
        mask = cv2.inRange(hsv, lower_range, upper_range)

        # You can also visualize the real part of the target color (Optional)
        res = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Converting the binary mask to 3 channel image, this is just so 
        # we can stack it with the others
        mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        # stack the mask, orginal frame and the filtered result
        stacked = np.hstack((mask_3,frame,res))

        # Run the detection formula
        corners, ids, rP = aruco.detectMarkers(mask_3, aruco_dict)

        # Calculate the pose of the markers
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 63, CM, dist_coef)
        # Draw the detected markers as an overlay
        out = aruco.drawDetectedMarkers(mask_3, corners, ids)

        # Run loop if ids are detected
        if ids is not None:
            for i, id in enumerate(ids):
                # Overlay the axis on the image
                out = aruco.drawAxis(out, CM, dist_coef, rvecs[i][0][:], tvecs[i][0][:], 30)
                
                # Only store the aruco coordinates if it hasn't been stored yet
                if ids[i][0] in ID:
                    pass
                else:
                    # Print the tvecs tranformation matrix or Aruco coordinates
                    # print("X = {:4.1f} Y = {:4.1f} Z = {:4.1f} ID = {:2d}".format(tvecs[i][0][0], tvecs[i][0][1], tvecs[i][0][2], ids[i][0]))
                    X.append(tvecs[i][0][0])
                    Y.append(tvecs[i][0][1])
                    Z.append(tvecs[i][0][2])
                    ID.append(ids[i][0])
                    rvecs_ap = np.array( [rvecs[i][0][:]] )
                    RVECS = np.append(RVECS, rvecs_ap, axis=0)


        # Display the original frame in a window and aruco markers
        cv2.imshow('frame-image', mask_3)


        # If the button q is pressed in one of the windows
        if cv2.waitKey(20) & 0xFF == ord('q'):
            # Exit the While loop
            break

    # When everything done, release the capture
    cap.release()
    # close all windows
    cv2.destroyAllWindows()
    # # exit the kernel
    # exit(0)

    # Remove the 1st index and format correctly    
    # RVECS = np.array([RVECS])
    RVECS = RVECS[:, np.newaxis, :]
    RVECS = np.delete(RVECS, 0, 0)

    return X, Y, Z, ID, RVECS

def initialScan():
    X, Y, Z, ID, rvecs = vision()

    # Ensure all coordinates are +ve
    X = [abs(ele) for ele in X]
    Y = [abs(ele) for ele in Y]
    Z = [abs(ele) for ele in Z]

    # plt.scatter(X, Y, marker='o')
    # plt.axis('equal')
    # plt.show()

    # Combine X(0), Y(1), Z(2) coordinates and ID(3) into P (point) variables
    P = []
    ID_count = len(ID)
    for i in range(ID_count):
        P.append(
            [ID[i], X[i], Y[i], Z[i]]
        )
    # print("P =", P)

    # Find the P value which corresponds to the Event (ID = 1)
    event_ind = [i for i, el in enumerate(P) if 1 in el][0]
    eventLocation = P[event_ind] # Store the event location to be outputted
    del P[event_ind] # Delete the event from further calculations
    eventRvec = rvecs[event_ind][0][:] # Store the event rotational vector
    rvecs = np.delete(rvecs, event_ind, 0) # Delete the eventRvec from the remaining rvecs

    # Find the P value which corresponds to the Robot (ID = 0)
    robot_ind = [i for i, el in enumerate(P) if 0 in el][0]

    ## Nearest neighbour sorting algorithm
    arucoLocations = [[]]
    P_deleted = P[:]
    del P_deleted[robot_ind]

    # Run until all locations have been visited 
    for x in range(ID_count-2):

        distance = [] 
        # Find the nearest ID and it's index
        if x == 0:
            for i in range(len(P)):
                distance.append(
                        math.sqrt( ((P[i][1] - P[robot_ind][1]) **2) + ((P[i][2] - P[robot_ind][2]) **2) )
                        ) #Compute using 2D Pythagoras
            min_distance = second_smallest(distance)
            min_ind = distance.index(min_distance)
            angle_ind = distance.index(min_distance)
            min_ID = P[min_ind][0]
            # Store the next nearest ID to the previous ID
            arucoLocations.append(P[min_ind][:])
            # Fix indexing for next round
            if min_ind > robot_ind:
                min_ind = min_ind - 1
        else:
            for i in range(len(P_deleted)):
                distance.append(
                        math.sqrt( ((P_deleted[i][1] - P_deleted[min_ind][1]) **2) + ((P_deleted[i][2] - P_deleted[min_ind][2]) **2) )
                        ) #Compute using 2D Pythagoras
            min_distance = second_smallest(distance)
            min_ind = distance.index(min_distance)
            min_ID = P_deleted[min_ind][0]
            # Store the next nearest ID to the previous ID
            arucoLocations.append(P_deleted[min_ind][:])


    # Initial nearest Aruco ID
    del arucoLocations[0]
    min_ID = arucoLocations[0][0]
    # print("The nearest ID is", min_ID)
    min_ind = angle_ind
    # print("min_ind =", min_ind)
    # print("robot_ind =", robot_ind)

    # Store the rvec's for the robot and nearest marker
    rob_rvec = rvecs[robot_ind][0][:]
    marker_rvec = rvecs[min_ind][0][:]
    output_rvecs = rvecs
    output_rvecs = np.delete(output_rvecs, robot_ind, 0)

    # Calculate the relative rotation about the Z axis between the robot ID and nearest ID (beta in notes)
    R_ref_to_cam = cv2.Rodrigues(rob_rvec)[0] #reference to camera
    R_test_to_cam = cv2.Rodrigues(marker_rvec)[0] #test to camera
    R_cam_to_ref = np.transpose(R_ref_to_cam) #inverse of reference to camera
    R_test_to_ref = np.matmul(R_test_to_cam,R_cam_to_ref) #test to reference
    angles_matrix = rotationMatrixToEulerAngles(R_test_to_ref) 
    beta = np.degrees(angles_matrix[2])
    beta = 0 - beta

    # Calculate the relative angle between the Robot ID axis and the nearest ID location (sigma in notes)
    delta_x = P[robot_ind][1] - P[min_ind][1]
    delta_y = P[min_ind][2] - P[robot_ind][2]

    if delta_x > 0:
        if delta_y > 0:
            # upper right
            alpha = np.degrees(math.atan( (delta_x) / (delta_y) ))
        else:
            # lower right
            alpha = np.degrees(math.atan( (-1 * delta_y) / (delta_x) )) + 90
    else:
        if delta_y > 0:
            # upper left
            alpha = np.degrees(math.atan( (delta_y) / (-1 * delta_x) )) + 270
        else:
            # lower left
            alpha = np.degrees(math.atan( (-1 * delta_x) / (-1 * delta_y) )) + 180

    # print("Alpha =", alpha, "degrees")

    # Combine beta and alpha above to calculate the movement direction needed by the robot (sigma in notes)
    angle = alpha - beta

    # Convert to counter clockwise motion if faster
    if angle > 180:
        angle = angle - 360

    return(arucoLocations, output_rvecs, eventLocation, eventRvec, angle, min_distance)

arucoLocations, arucoRvecs, eventLocation, eventRvec, angle, distance = initialScan()
print("Final output", arucoLocations, arucoRvecs, eventLocation, eventRvec, angle, distance)