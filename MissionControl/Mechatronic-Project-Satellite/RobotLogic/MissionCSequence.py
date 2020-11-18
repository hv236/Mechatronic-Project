###  Mission Control logic ###

# Import libraries


# Initialise variables
AngleReached = 0; #Field 4 MQTT
TargetReached = 0; #Field 5 MQTT

## MQTT Fields - BOBAmosFET
# Field 1: Angle
# Field 2: Distance
# Field 3: Command
# Field 4: AngleReached
# Field 5: DistanceReached
# Field 6: 
# Field 7:
# Field 8: ShipHeight

# Initialise all functions neededef

def scanArea ()

    # return all ArucoCode Locations and BB8 Location
    return array(X,Y), BB8position

def RouteCalculator ()


#####  START  #####
 
###  Satellite Reset ###


### Set the height of the ship ###
# Generate random number between 0 and 10cm for example 
shipHeight = 10

# Send Command to Ship
try: 
    # Publish the value (integer) as a string. All messages are strings
    client.publish(MainTopic+"8",str(shipHeight))
    # Plot in the terminal what we just did
    print("%s %d" % (MainTopic+"8", shipHeight))
    
    client.loop_stop()
    # Disconnect
    client.disconnect()

# Capture any other error from the three lines (most likely will be a publish error)
except:
    print ("There was an error while publishing the data.")

###  Initial Area Scanning ###

BB8_X = scanArea
BB8_Y = scanArea
ArucoCodeList = scanArea

n = index (ArucoCodeList)


while n => 0 

    ### Calculate closest ArucoCode from BB8

    Angle, Distance = RouteCalculator(ArucoCodeList, BB8_X, BB8_Y)

    ### Send Turning Command 

    C = 1
    A = Angle
    D = 0

    try: 
    # Publish the value (integer) as a string. All messages are strings
        client.publish(MainTopic+"1",str(A))
        client.publish(MainTopic+"2",str(D))
        client.publish(MainTopic+"3",str(C))

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




