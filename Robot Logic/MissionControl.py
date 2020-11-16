"""
This is the the BOBA mosFET Mission Control script. 

@author: Hélène Verhaeghe
"""

#import necessary libaries
import socket   # This library will allow you to communicate over the network
import time     # This library will allow us to access the system clock for pause/sleep/delay actions
import logging  # This library will offer us a different method to print information on the terminal (better for debugging purposes)
import paho.mqtt.client as mqtt # This is the library to do the MQTT communications
import time # This is the library that will allow us to use the sleep function

# MISSION CONTROL LOGIC WRITTEN
    #Receive confirmation from the ship, the distance and acceleration required for the RObot to go in ship ????
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


### Send Command to Robot Control ###

# After we connect we subsribe to one (or more) topics in this case the topic number 1
def on_connect(client,userdata,flags,rc):
    print ("Connected with result code "+str(rc))
    client.subscribe(MainTopic+"1")

# The callback for when a PUBLISH message is received from the server. I.e. when a new value for the topic we subscribed to above updates
# the expression str(int(msg.payload.rstrip(b'\x00'))) converts the message from the server to an integer and then a string for printing.
# Specifically:
#   msg.payload.rstrip(b'\x00') This is getting all the bytes from the server (should be 11 in our case) and removes bytes with the value \x00 from the right
#   int() converts the remaining bytes into an integer, it threats them as a string
#   str() converts the integer into a string for the purpose of printing it. See below (l.56) for an alternative way to do this
def on_message(client, userdata, msg):
    print(str(time.time())+" In topic: "+msg.topic+" the value was "+ str(int(msg.payload.rstrip(b'\x00'))))


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

# Start the client to enable the above events to happen
client.loop_start()

# Pick a Command, an Angle and a Distance
    # Angle in degrees
    # Distance in cm
Command = 1
Angle = 5 
Distance = 7 


try: 
    # Publish the value (integer) as a string. All messages are strings
    client.publish(MainTopic+"1",str(Angle))
    client.publish(MainTopic+"2",str(Distance))
    client.publish(MainTopic+"3",str(Command))
    # Plot in the terminal what we just did
    print("%s %d" % (MainTopic+"1", Angle))
    print("%s %d" % (MainTopic+"2", Distance))
    print("%s %d" % (MainTopic+"3", Command))

    client.loop_stop()
    # Disconnect
    client.disconnect()

# Capture any other error from the three lines (most likely will be a publish error)
except:
    print ("There was an error while publishing the data.")





 



