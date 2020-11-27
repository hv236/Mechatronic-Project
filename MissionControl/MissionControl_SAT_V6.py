"""
This is the the BOBA mosFET Mission Control script. 

@author: Hélène Verhaeghe
@Coauthor (Satellite): Jerôme De Saulles 
"""

#import necessary libaries
import cv2 # This is the vision library OpenCV
import numpy as np # This is a library for mathematical functions for python (used later)
import socket   # This library will allow you to communicate over the network
import time     # This library will allow us to access the system clock for pause/sleep/delay actions
import cv2.aruco as aruco #Import the AruCo library
import math # Import the math library
import itertools as it
import logging  # This library will offer us a different method to print information on the terminal (better for debugging purposes)
import paho.mqtt.client as mqtt # This is the library to do the MQTT communications
import time # This is the library that will allow us to use the sleep function
import random
import threading


# Initialise variables
AngleReached = 0 #Field 4 MQTT
DistanceReached = 0 #Field 5 MQTT
CommandCount_A = 0 #Keeping Track of number of turning command was send to BB8
CommandCount_D = 0 #Keeping Track of number of moving command was send to BB8
InPosition = 0
MagneticField = 0

print("CommandCount_A: "+str(CommandCount_A))
print("CommandCount_D: "+str(CommandCount_D))


## MQTT Fields - BOBAmosFET
# Field 1: Angle (Command)
# Field 2: Distance (Command)
# Field 3: Command (Command)
# Field 4: AngleReached (Response)
# Field 5: DistanceReached (Response)
# Field 6: MagneticField (Response)
# Field 7: Speed (Command) CURRENTLY NOT ACTIVE
# Field 8: ShipHeight (Command)

# Satellite functions

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

    t_end = time.time() + 10

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
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 95, CM, dist_coef)
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
  
def BB8_check(target_arucoLocations, target_rvecs, tolerance):
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
            math.sqrt( ((target_arucoLocations[i][1] - robot_loc[1]) ** 2) + 
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
        command = 3

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
        command = 3

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
        delta_x = robot_loc[1] - target_arucoLocations[0][1]
        delta_y = target_arucoLocations[0][2] - robot_loc[2]
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

def eventAngle(eventRvec):
    
    # Function to output the angle required for robot to face the event

    # Call the vision function
    X, Y, Z, ID, rvecs = vision()
    # Ensure all coordinates are +ve
    X = [abs(ele) for ele in X]
    Y = [abs(ele) for ele in Y]
    Z = [abs(ele) for ele in Z]

    # # Plot the found markers
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
    print("P =", P)

    # Find the P value which corresponds to the Robot (ID = 0)
    robot_ind = [i for i, el in enumerate(P) if 0 in el][0]

    # Store the rvec's for the robot and nearest marker
    rob_rvec = rvecs[robot_ind][0][:]
    marker_rvec = eventRvec

    # Calculate the relative rotation about the Z axis between the robot ID and nearest ID (beta in notes)
    R_ref_to_cam = cv2.Rodrigues(rob_rvec)[0] #reference to camera
    R_test_to_cam = cv2.Rodrigues(marker_rvec)[0] #test to camera
    R_cam_to_ref = np.transpose(R_ref_to_cam) #inverse of reference to camera
    R_test_to_ref = np.matmul(R_test_to_cam,R_cam_to_ref) #test to reference
    angles_matrix = rotationMatrixToEulerAngles(R_test_to_ref) 
    beta = np.degrees(angles_matrix[2])
    
    if beta > 180:
        beta = beta - 360

    beta = targetAngle
    Command = 1

    return(targetAngle, Command)

# Connect to MQTT Server

# After we connect we subsribe to one (or more) topics in this case the topic number 1
def on_connect(client,userdata,flags,rc):
    print ("Connected with result code "+str(rc))
    client.subscribe(MainTopic+"4") #AngleReached BB8 Response Channel
    client.subscribe(MainTopic+"5") #DistanceReached BB8 Response Channel
    client.subscribe(MainTopic+"6") #MagneticField BB8 Response Channel

# The callback for when a PUBLISH message is received from the server. I.e. when a new value for the topic we subscribed to above updates
def on_message(client, userdata, msg):
    global Check
    global MagnetCheck
    global InPosition
    global TargetAngle
    global TargetDistance
    global MagneticField
    global CommandCount_A
    global CommandCount_D
    global Command
    
    print(str(time.time())+" In topic: "+msg.topic+" the value was "+ str(int(msg.payload.rstrip(b'\x00'))))

    
    data = int(msg.payload.rstrip(b'\x00'))

    if msg.topic == "BOBAmosFET/4":
         
        AngleReached = data
        
        if AngleReached == CommandCount_A + 1:
            
            Command = 2 #Satellite Function output
            print("Command value change to "+str(Command))

            CommandCount_A = CommandCount_A + 1 # Set to next command index
            print("CommandCount_A: "+str(CommandCount_A))

    elif msg.topic == "BOBAmosFET/5":
        
        DistanceReached = data
        
        if DistanceReached == CommandCount_D + 1:

            Check = 1 
            print("Check value change to "+str(Check))

            InPosition, TargetAngle, TargetDistance, Command = BB8_check(target_arucoLocations, target_rvecs, tolerance)
            #print('InPosition, TargetAngle and TargetDistance =', InPosition, TargetAngle, TargetDistance)

            #InPosition = 1
            print("InPosition value change to "+str(InPosition))
            #TargetAngle = 0 #Satellite Function output
            print("TargetAngle value change to "+str(TargetAngle))
            #TargetDistance = 0 #Satellite Function output
            print("TargetDistance value change to "+str(TargetDistance))
            #Command = 3  #Satellite Function output
            print("Command value change to "+str(Command))

            CommandCount_D= CommandCount_D + 1 # Set to next command index
            print("CommandCount_D: "+str(CommandCount_D))

    elif msg.topic == "BOBAmosFET/7":
        
        MagneticFieldDetected = data
        
        if MagneticFieldDetected == 1:
            
            MagnetCheck = 1 
            print("Presence of a magnet has been checked")
            MagneticField = 1 

        else: 
            
            MagnetCheck = 1 
            print("Presence of a magnet has been checked")
        
            MagneticField = 0
            print("Nothing found, go to next location")            
    
# Create the mqtt client object
client = mqtt.Client() 
# Assign the function for the connection event
client.on_connect = on_connect
# Assign the function for the new message event
client.on_message = on_message

# Set the username and password
client.username_pw_set("student",password="smartPass")

# Connect to the server using a specific port with a timeout delay (in seconds)
client.connect("ec2-3-10-235-26.eu-west-2.compute.amazonaws.com",31415,60)

# Create your main topic string. Everything else should be fields with values 1-8
MainTopic = "BOBAmosFET/"

# Start the client
client.loop_start() 

################################  START  ################################

############################# INITIAL MODE ##############################

### Send Command to Mill.Falcon ###

# Generate random number between 14 and 24
shipHeight = random.randint(15, 23)
Command = 8
print("Ship height is set to ", shipHeight)

# Publish the value (integer) as a string. All messages are strings
client.publish(MainTopic+"3",str(Command))
client.publish(MainTopic+"8",str(shipHeight))
print("%s %d" % (MainTopic+"8", shipHeight))


### Initial Scan ### 

arucoLocations, arucoRvecs, eventLocation, eventRvec, Targetangle, Targetdistance = initialScan()
print("Final output", arucoLocations, arucoRvecs, eventLocation, eventRvec, angle, distance)

for i,_ in enumerate(arucoLocations):
    
    print("Target Aruco =", _[0])
    print("Target Angle =", TargetAngle)
    print("Target Distance =", TargetDistance)

    if i < (len(arucoLocations)-1):

        # When we are not on the last Arucolocation, determine target and next arucolocation target

        target_arucoLocations = [[]]
        target_arucoLocations[0][:] = arucoLocations[i][:]
        target_arucoLocations.append(arucoLocations[i+1][:])
        print('target_arucoLocations =', target_arucoLocations)

        target_rvecs = np.array([[arucoRvecs[i][0][:]],[arucoRvecs[i+1][0][:]] ])
        # print('target_rvecs =', target_rvecs)
        tolerance = 50 # Distance tolerance to target aruco (in mm)

    else:
        # if on last location, determine only the target location
        target_arucoLocations = [[]]
        target_arucoLocations[0][:] = arucoLocations[i][:]
        print('target_arucoLocations =', target_arucoLocations)

        target_rvecs = np.array([ [arucoRvecs[i][0][:]] ])
        # print('target_rvecs =', target_rvecs)
        tolerance = 50 # Distance tolerance to target aruco (in mm)

    # Initialise variables 
    InPosition = 0
    Check = 1
    Command = 1
    #Speed = 45
    MagnetCheck = 0

    while InPosition < 1:
        # While not in position

            ############################# TURNING MODE ##############################
            if Check == 1:

                print("Entered TURNING MODE")
                
                ### Send Command to BB8
                Angle = TargetAngle
                Distance = 0

                # Publish the value (integer) as a string. All messages are strings
                client.publish(MainTopic+"1",str(Angle))
                client.publish(MainTopic+"2",str(Distance))
                client.publish(MainTopic+"3",str(Command))
                

                # Plot in the terminal what we just did
                print("%s %d" % (MainTopic+"1", Angle))
                print("%s %d" % (MainTopic+"2", Distance))
                print("%s %d" % (MainTopic+"3", Command))
                

                # Reset Command value
                Command = 0
                Check = 0
            
            
                ### Wait for a signal from BB8. Once signal received, go to MOVING MODE.
                while Command == 0:
                    time.sleep(10)
                    print("waiting for a signal from BB8")
                    pass

                ############################# MOVING MODE ##############################
                print("Entered MOVING MODE")
                
                if Command == 2:
                    
                    ### Send Command to BB8
                    Angle = 0
                    Distance = TargetDistance

                    # Publish the value (integer) as a string. All messages are strings
                    client.publish(MainTopic+"1",str(Angle))
                    client.publish(MainTopic+"2",str(Distance))
                    client.publish(MainTopic+"3",str(Command))
                    #client.publish(MainTopic+"7",str(Speed))

                    # Plot in the terminal what we just did
                    print("%s %d" % (MainTopic+"1", Angle))
                    print("%s %d" % (MainTopic+"2", Distance))
                    print("%s %d" % (MainTopic+"3", Command))
                    #print("%s %d" % (MainTopic+"7", Speed))

                    Command = 0

                    ### Once signal from BB8, position-check function will be triggered within on_message.
                    while Command == 0:
                        time.sleep(10)
                        print("waiting for a signal from BB8")
                        pass
                    

            ############################# DETECTING MODE ##############################
            print("Entered DETECTING MODE")

            # Wait for a signal from BB8. 
            while MagnetCheck == 0:
                time.sleep(10)
                print("waiting for a signal from BB8")
                pass

            # If no Magnet detected, keep looping. 
            # If a magnet detected, go to Event Mode.     
            if MagneticField == 1:
                print("LightSaber found, go to ship")
                break
    

############################# EVENT MODE ##############################
print("Entered EVENT MODE")
 
# Determine Target-Angle and -Distance to reach the ship   
InPosition, TargetAngle, TargetDistance, Command = BB8_check(eventLocation, eventRvec, tolerance) 

 
Check = 1
Command = 1

while InPosition < 1:
    # While not in position
            ############################# TURNING MODE ##############################
            if Check == 1:

                print("Entered TURNING MODE")
                
                ### Send Command to BB8
                Angle = TargetAngle
                Distance = 0

                # Publish the value (integer) as a string. All messages are strings
                client.publish(MainTopic+"1",str(Angle))
                client.publish(MainTopic+"2",str(Distance))
                client.publish(MainTopic+"3",str(Command))
                

                # Plot in the terminal what we just did
                print("%s %d" % (MainTopic+"1", Angle))
                print("%s %d" % (MainTopic+"2", Distance))
                print("%s %d" % (MainTopic+"3", Command))
                
                Command = 0
                Check = 0
            
            
                ###  Wait for a signal from BB8. Once signal received, go to MOVING MODE.
                while Command == 0:
                    time.sleep(10)
                    print("waiting for a signal from BB8")
                    pass

                ############################# MOVING MODE ##############################
                print("Entered MOVING MODE")
                if Command == 2:

                    ### Send Command to BB8
                    Angle = 0
                    Distance = TargetDistance

                    # Publish the value (integer) as a string. All messages are strings
                    client.publish(MainTopic+"1",str(Angle))
                    client.publish(MainTopic+"2",str(Distance))
                    client.publish(MainTopic+"3",str(Command))

                    # Plot in the terminal what we just did
                    print("%s %d" % (MainTopic+"1", Angle))
                    print("%s %d" % (MainTopic+"2", Distance))
                    print("%s %d" % (MainTopic+"3", Command))

                    Command = 0
                    
                    
                    ### Once signal from BB8, position-check function will be triggered within on_message
                    while Command == 0:
                        time.sleep(10)
                        print("waiting for a signal from BB8")
                        pass



############################# ARRIVED TO SHIP ##############################
print("Arrived to ship")

### Rotate BB8 to have him face the ship 

while TargetAngle >= 5:
    #Keep looping until he is the right position to board the ship

    # Determine the Angle BB8 needs to rotate to board
    TargetAngle, Command = eventAngle()
    
    ### Send Command to BB8
    Angle = TargetAngle
    Distance = 0

    # Publish the value (integer) as a string. All messages are strings
    client.publish(MainTopic+"1",str(Angle))
    client.publish(MainTopic+"2",str(Distance))
    client.publish(MainTopic+"3",str(Command))

    # Plot in the terminal what we just did
    print("%s %d" % (MainTopic+"1", Angle))
    print("%s %d" % (MainTopic+"2", Distance))
    print("%s %d" % (MainTopic+"3", Command))

    Command = 0
    Check = 0


    ### Wait for a signal from BB8. 
    while Command == 0:
        time.sleep(10)
        print("waiting for a signal from BB8")
        pass


############################# BOARDING ##############################
print("BB8 in position to board the ship")


#Speed = 75
TargetDistance = 100 ###TOOOOO
        
### Send Command to BB8
Command = 2
Angle = 0
Distance = TargetDistance

# Publish the value (integer) as a string. All messages are strings
client.publish(MainTopic+"1",str(Angle))
client.publish(MainTopic+"2",str(Distance))
client.publish(MainTopic+"3",str(Command))
#client.publish(MainTopic+"7",str(Speed))

# Plot in the terminal what we just did
print("%s %d" % (MainTopic+"1", Angle))
print("%s %d" % (MainTopic+"2", Distance))
print("%s %d" % (MainTopic+"3", Command))
#print("%s %d" % (MainTopic+"7", Speed))


Command = 0

### Once signal from BB8, position-check function will be triggered within on_message
while Command == 0:
    time.sleep(10)
    print("waiting for a signal from BB8")
    pass     

print("BB8 should have boarded the ship")

client.loop_stop()
# Disconnect
client.disconnect()
