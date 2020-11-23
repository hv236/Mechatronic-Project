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

print("CommandCount_A: "+str(CommandCount_A))
print("CommandCount_D: "+str(CommandCount_D))


## MQTT Fields - BOBAmosFET
# Field 1: Angle
# Field 2: Distance
# Field 3: Command
# Field 4: AngleReached
# Field 5: DistanceReached
# Field 6: MagneticField 
# Field 7: 
# Field 8: ShipHeight

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

def initialScan():
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


# Connect to MQTT Server

# After we connect we subsribe to one (or more) topics in this case the topic number 1
def on_connect(client,userdata,flags,rc):
    print ("Connected with result code "+str(rc))
    client.subscribe(MainTopic+"4")
    client.subscribe(MainTopic+"5")
    

# The callback for when a PUBLISH message is received from the server. I.e. when a new value for the topic we subscribed to above updates
def on_message(client, userdata, msg):
    global Check
    global InPosition
    global TargetAngle
    global TargetDistance
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

        
            print("InPosition value change to "+str(InPosition))
            #TargetAngle = 0 #Satellite Function output
            print("TargetAngle value change to "+str(TargetAngle))
            #TargetDistance = 0 #Satellite Function output
            print("TargetDistance value change to "+str(TargetDistance))
            #Command = 3  #Satellite Function output
            print("Command value change to "+str(Command))

            CommandCount_D= CommandCount_D + 1 # Set to next command index
            print("CommandCount_D: "+str(CommandCount_D))
    #else:
        #pass 

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

# Generate random number between 0 and 10
shipHeight = random.randint(0, 10)
#print("Random integer from 0 to 10")
#print("Random integer: ", shipHeight)


# Publish the value (integer) as a string. All messages are strings
client.publish(MainTopic+"8",str(shipHeight))
# Plot in the terminal what we just did
print("%s %d" % (MainTopic+"8", shipHeight))


### Initial Scan ### 

#TargetAngle = 45
#TargetDistance = 100    
arucoLocations, arucoRvecs, TargetAngle, TargetDistance = initialScan()
#arucoRvecs = [[[-2.65077743, 0.01517437, 0.0167672 ]],[[ 3.45453988, -0.04192978, 0.42548113]]]
#arucoLocations = [[11, 27.951521361774212, 88.53147453041412, 682.1787133172342], [13, 107.79970108395187, 105.40086628164487, 742.4444729628245]]

for i,_ in enumerate(arucoLocations):

    if i < len(arucoLocations)-1:

        target_arucoLocations = [[]]
        target_arucoLocations[i][:] = arucoLocations[i][:]
        target_arucoLocations.append(arucoLocations[i+1][:])
        print('target_arucoLocations =', target_arucoLocations)

        target_rvecs = np.array([[arucoRvecs[i][0][:]],[arucoRvecs[i+1][0][:]] ])
        # print('target_rvecs =', target_rvecs)
        tolerance = 50 # Distance tolerance to target aruco (in mm)

    else:
        target_arucoLocations = [[]]
        target_arucoLocations[i][:] = arucoLocations[i][:]
        print('target_arucoLocations =', target_arucoLocations)

        target_rvecs = np.array([[arucoRvecs[i][0][:]] ])
        # print('target_rvecs =', target_rvecs)
        tolerance = 50 # Distance tolerance to target aruco (in mm)

    InPosition = 0
    Check = 1
    Command = 1


    while InPosition < 1:

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
            
            
                ### 3. Wait for a signal from BB8. Once signal received, go to MOVING MODEs
                while Command == 0:
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
                    ### 3. Once signal from BB8, position-check function will be triggered within on_message
                    
                    while Command == 0:
                        print("waiting for a signal from BB8")
                        pass
                    

    ############################# DETECTING MODE ##############################
    print("Entered DETECTING MODE")





client.loop_stop()
# Disconnect
client.disconnect()
