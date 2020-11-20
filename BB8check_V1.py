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

def initialScan():

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
        Camera = np.load('Calibrated_Rig_Camera.npz')
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

        # Position the window
        cv2.moveWindow("frame-image", 0, 0)

        t_end = time.time() + 1

        # Execute this continuously
        while time.time() < t_end:
            # Capture current frame from the camera
            ret, frame = cap.read()

            # Convert the image from the camera to Gray scale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Run the detection formula
            corners, ids, rP = aruco.detectMarkers(gray, aruco_dict)

            # # Count the number of Arucos visible
            # try:
            #     IDScount = len(ids)
            # except:
            #     IDScount = 0

            # Calculate the pose of the markers
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 53, CM, dist_coef) # <<<< IMPORTANT: number needs changing to width of printed arucos (in mm)
            # Draw the detected markers as an overlay
            out = aruco.drawDetectedMarkers(frame, corners, ids)

            # Create Coordinate Storage Arrays
            X = [] #X Coordinate Locations Array
            Y = []
            Z = []
            ID = []

            # Run loop if ids are detected
            if ids is not None:
                for i, id in enumerate(ids):
                    # Overlay the axis on the image
                    out = aruco.drawAxis(out, CM, dist_coef, rvecs[i][0][:], tvecs[i][0][:], 30)
                    # Print the tvecs tranformation matrix or Aruco coordinates
                    # print("X = {:4.1f} Y = {:4.1f} Z = {:4.1f} ID = {:2d}".format(tvecs[i][0][0], tvecs[i][0][1], tvecs[i][0][2], ids[i][0]))
                    X.append(tvecs[i][0][0])
                    Y.append(tvecs[i][0][1])
                    Z.append(tvecs[i][0][2])
                    ID.append(ids[i][0])
                    # debugTEST = []
        
            
            # Display the original frame in a window and aruco markers
            cv2.imshow('frame-image', frame)


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

        return X, Y, Z, ID, rvecs

    X, Y, Z, ID, rvecs = vision()

    # Ensure all coordinates are +ve
    X = [abs(ele) for ele in X]
    Y = [abs(ele) for ele in Y]
    Z = [abs(ele) for ele in Z]

    # Combine X(0), Y(1), Z(2) coordinates and ID(3) into P (point) variables
    P = []
    ID_count = len(ID)
    for i in range(ID_count):
        P.append(
            [ID[i], X[i], Y[i], Z[i]]
        )
    # print(P)

    # Find the P value which corresponds to the Robot (ID = 0)
    robot_ind = [i for i, el in enumerate(P) if 0 in el][0]

    distance = []
    # Count the distances between the robot and the other IDs
    for i in range(ID_count):
        # print(P[i][1], P[robot_ind][1])
        distance.append(
            math.sqrt( ((P[i][1] - P[robot_ind][1]) **2) + ((P[i][2] - P[robot_ind][2]) **2) )
            ) #Compute using 2D Pythagoras

    # print("Distance vector =", distance)
    min_distance = second_smallest(distance)
    min_ind = distance.index(min_distance)
    min_ID = P[min_ind][0]
    print("The nearest ID is", min_ID)
    # print("The distance to ID", min_ID, "from the robot is", min_distance, "mm")

    # Store the rvec's for the robot and nearest marker
    rob_rvec = rvecs[robot_ind][0][:]
    marker_rvec = rvecs[min_ind][0][:] # Replace 2 with min_ind when working properly
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

    # Rewrite the aruco locations with the robot location removed
    # ADD LOGICAL SORTING FUNCTION HERE TO ARRANGE ARUCOS IN ORDER THEY SHOULD BE VISITED
    arucoLocations = P
    del arucoLocations[robot_ind]

    return(arucoLocations, output_rvecs, angle, min_distance)

arucoLocations, arucoRvecs, angle, distance = initialScan()
print('Angle and distance =', angle, distance)

target_arucoLocations = [[]]
target_arucoLocations[0][:] = arucoLocations[0][:]
target_arucoLocations.append(arucoLocations[1][:])
print('target_arucoLocations =', target_arucoLocations)

target_rvecs = np.array([[arucoRvecs[0][0][:]],[arucoRvecs[1][0][:]] ])
# print('target_rvecs =', target_rvecs)
tolerance = 30 # Distance tolerance to target aruco (in mm)

input("Press Enter to continue...")

def BB8_check(target_arucoLocations, target_rvecs, tolerance):

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

        m1, m2 = float('inf'), float('inf')
        for x in numbers:
            if x <= m1:
                m1, m2 = x, m1
            elif x < m2:
                m2 = x
        return m2

    def vision():

        # Load the camera calibration values
        Camera = np.load('Calibrated_Rig_Camera.npz')
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

        # Position the window
        cv2.moveWindow("frame-image", 0, 0)

        t_end = time.time() + 5

        # Execute this continuously
        while time.time() < t_end:
            # Capture current frame from the camera
            ret, frame = cap.read()

            # Convert the image from the camera to Gray scale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Run the detection formula
            corners, ids, rP = aruco.detectMarkers(gray, aruco_dict)

            # Calculate the pose of the markers
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 53, CM, dist_coef) # <<<< IMPORTANT: number needs changing to width of printed arucos (in mm)
            # Draw the detected markers as an overlay
            out = aruco.drawDetectedMarkers(frame, corners, ids)

            # Create Coordinate Storage Arrays
            X = [] #X Coordinate Locations Array
            Y = []
            Z = []
            ID = []

            # Run loop if ids are detected
            if ids is not None:
                for i, id in enumerate(ids):
                    # Overlay the axis on the image
                    out = aruco.drawAxis(out, CM, dist_coef, rvecs[i][0][:], tvecs[i][0][:], 30)
                    # Print the tvecs tranformation matrix or Aruco coordinates
                    # print("X = {:4.1f} Y = {:4.1f} Z = {:4.1f} ID = {:2d}".format(tvecs[i][0][0], tvecs[i][0][1], tvecs[i][0][2], ids[i][0]))
                    X.append(tvecs[i][0][0])
                    Y.append(tvecs[i][0][1])
                    Z.append(tvecs[i][0][2])
                    ID.append(ids[i][0])
                    # debugTEST = []
        
            
            # Display the original frame in a window and aruco markers
            cv2.imshow('frame-image', frame)


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

        return X, Y, Z, ID, rvecs

    X, Y, Z, ID, rvecs = vision()

    # Ensure all coordinates are +ve
    X = [abs(ele) for ele in X]
    Y = [abs(ele) for ele in Y]
    Z = [abs(ele) for ele in Z]

    # Combine X(0), Y(1), Z(2) coordinates and ID(3) into P (point) variables
    P = []
    ID_count = len(ID)
    for i in range(ID_count):
        P.append(
            [ID[i], X[i], Y[i], Z[i]]
        )
    # print(P)

    # Find the P value which corresponds to the Robot (ID = 0)
    robot_ind = [i for i, el in enumerate(P) if 0 in el][0]
    robot_loc = P[robot_ind]
    robot_rvec = rvecs[robot_ind][0][:]

    distance = []
    # Count the distances between the robot and the target marker
    for i in range(len(target_arucoLocations)):
        # print(P[i][1], P[robot_ind][1])
        distance.append(
            math.sqrt( ((target_arucoLocations[i][1] - robot_loc[1]) **2) + 
            ((target_arucoLocations[i][2] - robot_loc[2]) **2) )
            ) #Compute using 2D Pythagoras
        print('Distance i =', distance[i])


    # Define the acceptable tolerance from the target aruco location (in mm)
    dist_tol = tolerance
    # Logic for calculating either corrected target angle+distance or next target angle+distance
    if distance[0] < dist_tol and len(distance) == 1: # Target reached, final marker
        target_angle = 0
        target_distance = 0
        state = 1
        command = 0

    elif distance[0] < dist_tol and len(distance) == 2: # Target reached, move onto next marker
        # Calculate angle to next target aruco
        marker_rvec = target_rvecs[1][0][:] # Define target rvec

        # Calculate the relative rotation about the Z axis between the robot ID and nearest ID (beta in notes)
        R_ref_to_cam = cv2.Rodrigues(robot_rvec)[0] #reference to camera
        R_test_to_cam = cv2.Rodrigues(marker_rvec)[0] #test to camera
        R_cam_to_ref = np.transpose(R_ref_to_cam) #inverse of reference to camera
        R_test_to_ref = np.matmul(R_test_to_cam,R_cam_to_ref) #test to reference
        angles_matrix = rotationMatrixToEulerAngles(R_test_to_ref) 
        beta = np.degrees(angles_matrix[2])
        beta = 0 - beta

        # Calculate the relative angle between the Robot ID axis and the nearest ID location (sigma in notes)
        delta_x = robot_loc[1] - target_arucoLocations[1][1]
        delta_y = target_arucoLocations[1][2] - robot_loc[2]
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
        # Combine beta and alpha above to calculate the movement direction needed by the robot (sigma in notes)
        target_angle = alpha - beta
        # Convert to counter clockwise motion if faster
        if target_angle > 180:
            target_angle = target_angle - 360

        # Output the target distance
        target_distance = distance[1]
        state = 1
        command = 0

    elif distance[0] > dist_tol: # Target missed, recalculate angle to current target
        # Calculate angle to current target aruco
        # Calculate angle to next target aruco
        marker_rvec = target_rvecs[0][0][:] # Define target rvec

        # Calculate the relative rotation about the Z axis between the robot ID and nearest ID (beta in notes)
        R_ref_to_cam = cv2.Rodrigues(robot_rvec)[0] #reference to camera
        R_test_to_cam = cv2.Rodrigues(marker_rvec)[0] #test to camera
        R_cam_to_ref = np.transpose(R_ref_to_cam) #inverse of reference to camera
        R_test_to_ref = np.matmul(R_test_to_cam,R_cam_to_ref) #test to reference
        angles_matrix = rotationMatrixToEulerAngles(R_test_to_ref) 
        beta = np.degrees(angles_matrix[2])
        beta = 0 - beta

        # Calculate the relative angle between the Robot ID axis and the nearest ID location (sigma in notes)
        delta_x = robot_loc[1] - target_arucoLocations[1][1]
        delta_y = target_arucoLocations[1][2] - robot_loc[2]
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
        # Combine beta and alpha above to calculate the movement direction needed by the robot (sigma in notes)
        target_angle = alpha - beta
        # Convert to counter clockwise motion if faster
        if target_angle > 180:
            target_angle = target_angle - 360

        # Output the target distance
        target_distance = distance[0]
        state = 0
        command = 1

    


    return(state, target_angle, target_distance, command)

InPosition, angle, distance, command = BB8_check(target_arucoLocations, target_rvecs, tolerance)
print('State, angle and distance =', InPosition, angle, distance)