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
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 75, CM, dist_coef)
        # Draw the detected markers as an overlay
        out = aruco.drawDetectedMarkers(mask_3, corners, ids)

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

    return X, Y, Z, ID, rvecs