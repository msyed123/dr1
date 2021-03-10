#!/usr/bin/env python

import math
import time

import cv2
import cv2.aruco as aruco
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Bool

# --- Define Tag
id_to_find = 57  # - Change this to whatever the marker ID selected for the target happens to be
marker_size = 10  # - [cm]

# --- Spin up the ROS node and the publisher
positionPublisher = rospy.Publisher("dr1/target", PoseStamped, queue_size=1)  # Publish only the latest position data to the node
targetAcquisitionPublisher = rospy.Publisher("dr1/targetAcquired", Bool, queue_size=1)  # Zero the velocity if no target found
landingCommanderPub = rospy.Publisher('dr1/landing_commander_trigger', Bool, queue_size=1)
rospy.init_node("aruco_node")
# time.sleep(15)  # Let the node spin up before publishing to it


# ------------------------------------------------------------------------------
# ------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
# ------------------------------------------------------------------------------
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


# --- Get the camera calibration path
calib_path = "/home/dr1/catkin_workspace/src/dr1/scripts/c920/"
camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

# --- 180 deg rotation matrix around the x axis
R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

# --- Define the aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

# --- Capture the video camera (this may also be a video or a picture)
cap = cv2.VideoCapture(0)
# -- Set the camera size as the one it was calibrated with
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# -- Font for the text in the image
font = cv2.FONT_HERSHEY_SIMPLEX

while True:

    # -- Read the camera frame
    ret, frame = cap.read()

    # -- Convert in gray scale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # -- remember, OpenCV stores color images in Blue, Green, Red

    # -- Find all the aruco markers in the image
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                                                 cameraMatrix=camera_matrix, distCoeff=camera_distortion)

    if ids is not None and ids[0] == id_to_find:
        # -- ret = [rvec, tvec, ?]
        # -- array of rotation and position of each marker in camera frame
        # -- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
        # -- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

        # -- Unpack the output, get only the first
        rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

        # -- Draw the detected marker and put a reference frame over it
        aruco.drawDetectedMarkers(frame, corners)
        aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

        # -- Print the tag position in camera frame
        str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f" % (tvec[0], tvec[1], tvec[2])
        cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # -- Obtain the rotation matrix tag->camera
        R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc = R_ct.T

        # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
        roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip * R_tc)

        # -- Print the marker's attitude respect to camera frame
        str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
            math.degrees(roll_marker), math.degrees(pitch_marker),
            math.degrees(yaw_marker))
        cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Since the size of the marker is defined in cm and the flight computer responds to commands in meters, scale
        xPos = -1.0 * tvec[0] / 100
        yPos = tvec[1] / 100
        zPos = tvec[2] / 100  # For the time being, Z offset will be excluded from the ROS message to restrict commands to strictly planar moves

        # Construct the ROS message and publish
        relativePosition = PoseStamped()

        relativePosition.header = Header()
        relativePosition.header.stamp = rospy.Time.now()
        relativePosition.header.frame_id = "LOCAL_ENU"
        relativePosition.pose.position.x = xPos
        relativePosition.pose.position.y = yPos
        relativePosition.pose.position.z = zPos

        positionPublisher.publish(relativePosition)
        targetAcquisitionPublisher.publish(True)
        landingCommanderPub.publish(True)
    else:
        targetAcquisitionPublisher.publish(False)
        landingCommanderPub.publish(False)

        """
        Camera attitude determination. IMU will do this with superior accuracy.
        # Determine the attitude and orientation of the camera with respect to the marker
        pos_camera = -R_tc * np.matrix(tvec).T

        str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f" % (pos_camera[0], pos_camera[1], pos_camera[2])
        cv2.putText(frame, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # -- Get the attitude of the camera respect to the frame
        roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip * R_tc)
        str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
        math.degrees(roll_camera), math.degrees(pitch_camera),
        math.degrees(yaw_camera))
        cv2.putText(frame, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        """

    # # --- Display the frame
    # cv2.imshow('frame', frame)
    #
    # # --- use 'q' to quit
    # key = cv2.waitKey(1) & 0xFF
    # if key == ord('q'):
    #     cap.release()
    #     cv2.destroyAllWindows()
    #     break
