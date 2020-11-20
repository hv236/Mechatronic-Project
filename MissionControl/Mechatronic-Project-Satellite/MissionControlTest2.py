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

# Initialise variables
AngleReached = 0 #Field 4 MQTT
DistanceReached = 0 #Field 5 MQTT
CommandCount = 0
InPosition = 0

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

    # Position the window
    cv2.moveWindow("frame-image", 0, 0)

    t_end = time.time() + 1000

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
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 53, CM, dist_coef)
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



    #Receive confirmation from the satellite that camera is calibrated (Satellite MQTT)

    # Any other information that needs to be confirmed?
    # when all ready to go - let's start!

    # Receive from satellite, the number of ArucoCodes, n, and ship locations 

    # for i=[0,n]

    # Receive from the satellite, the closest ArucoCode from the Robot Location. 
                # as an angle, and distance?? 
                # or as x,y coordinates related to robot position??
 
 
    # Determine the path to get the robot there (if needed)
 
    #  Send the commands to move the Robot to AruCo accroding to the path
    # while the robot is moving, ask to satellite for Robot_Position every 2sec
        ### Find a way to check that is the correct location???

    # When Robot reached destination, send command to scan the area. 

        # If MagneticField = True 
        # Send command to Robot to turn on Green LED light 
        # break

        # else 
            # Send command to Robot to turn on Red LED light 
        
            # i = i+1;



    # Send command to Robot to go to ship location
    # while moving, check location
    # while the robot is moving, ask to satellite for Robot_Position every 2sec

    # When Robot reached destination 
            # Change rpm parameters
            # Send command to Robot to move to top of ship 

#def InitialScan ()
    
    # return all ArucoCode Locations and BB8 Location
    # return first arucoCode needed angle and distance 
    # (Keep track of ArucoCodeNBR)
     

#def CheckBB8_Angle (Function)
    # return PositionCheck (Function: angle or distance to check)
    # return calculated Angle and distance 

    #return inPosition, Angle, Distance 
    # inPOsition= 1 - meaning its in position
    # inPosition= 0 - meaning its not in position

#def CheckBB8_Distance (Function)
    # return PositionCheck (Function: angle or distance to check)
    # return calculated Angle and distance 

    #return inPosition, Angle, Distance 
    # inPOsition= 1 - meaning its in position
    # inPosition= 0 - meaning its not in position
# Connect to MQTT Server

# After we connect we subsribe to one (or more) topics in this case the topic number 1
def on_connect(client,userdata,flags,rc):
    print ("Connected with result code "+str(rc))
    client.subscribe(MainTopic+"4")
    client.subscribe(MainTopic+"5")
    

# The callback for when a PUBLISH message is received from the server. I.e. when a new value for the topic we subscribed to above updates
def on_message(client, userdata, msg):
    global AngleCheck
    global InPosition
    global TargetAngle
    global CommandCount_A
    global CommandCount_D
    global Command
    
    print(str(time.time())+" In topic: "+msg.topic+" the value was "+ str(int(msg.payload.rstrip(b'\x00'))))

    
    data = int(msg.payload.rstrip(b'\x00'))

    if msg.topic == "BOBAmosFET/4":
         
        AngleReached = data
        
        if AngleReached == CommandCount_A + 1:
            
            AngleCheck = 1 
            print("InPosition value change to "+str(InPosition))
            
            Command = 2
            print("Command value change to "+str(Command))

            CommandCount_A = CommandCount_A + 1 # Set to next command index
            print("CommandCount: "+str(CommandCount))

    elif msg.topic == "BOBAmosFET/5":
        
        DistanceReached = data
        
        if DistanceReached == CommandCount_D + 1:

            InPosition = 0 
            print("InPosition value change to"+str(InPosition))
            
            TargetAngle = 10
            print("TargetAngle value change to"+str(TargetAngle))

            TargetDistance = 34
            print("TargetDistance value change to"+str(TargetDistance))

            Command = 1
            print("Command value change to "+str(Command))

            CommandCount_D= CommandCount_D + 1 # Set to next command index
            print("CommandCount: "+str(CommandCount))
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


# Generate first commands requirement for going to closest ArucoCode
TargetAngle = 45
TargetDistance = 100    



InPosition = 0
AngleCheck = 0
Command = 1


while InPosition < 1:

        ############################# TURNING MODE ##############################
        while AngleCheck < 1:

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
        ### 3. Once signal from BB8, position-check function will be triggered within on_message 
            
        time.sleep(10)  # Delays for 10 seconds


        ############################# MOVING MODE ##############################

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
            
        time.sleep(10)  # Delays for 10 seconds

############################# DETECTING MODE ##############################


### Send Command to BB8

    #Command = 2
    #Angle = 0
    #Distance = TargetDistance

    # Publish the value (integer) as a string. All messages are strings
    #client.publish(MainTopic+"1",str(Angle))
    #client.publish(MainTopic+"2",str(Distance))
    #client.publish(MainTopic+"3",str(Command))

    # Plot in the terminal what we just did
    #print("%s %d" % (MainTopic+"1", Angle))
    #print("%s %d" % (MainTopic+"2", Distance))
    #print("%s %d" % (MainTopic+"3", Command))

    ### 3. Once signal from BB8, position-check function will be triggered within on_message

    #time.sleep(10)  # Delays for 10 seconds



client.loop_stop()
# Disconnect
client.disconnect()
