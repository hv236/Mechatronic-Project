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
