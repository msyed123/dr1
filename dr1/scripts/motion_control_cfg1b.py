#!/usr/bin/env python

import time
import numpy
import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32, String, Bool
from nav_msgs.msg import Odometry

freq = 20.0     # hz
dt = 1/freq

error = numpy.zeros(3)
last_error = numpy.zeros(3)
error_sum = numpy.zeros(3)
velocity = numpy.zeros(3)

vel = TwistStamped()
localPose = PoseStamped()

targetAcquired = False
landingFlag = False

kp = rospy.get_param("/vel_control_kp")
kd = rospy.get_param("/vel_control_kd")
ki = rospy.get_param("/vel_control_ki")

altitudeSetpoint = rospy.get_param("/altitude_setpoint")
descentRate = rospy.get_param("/descent_rate")
maximumSpeed = rospy.get_param("/max_speed")

def pubVelSetpoint(velX, velY, velZ):
    global vel
    vX, vY, vZ = velX, velY, velZ
    mag = math.sqrt((vX*vX) + (vY*vY) + (vZ*vZ))

    if mag > maximumSpeed:
        vX, vY, vZ = 0, 0, 0
        rospy.logerr("Velocity command exceeds maximum allowable. Sending zero velocity vector.")

    vel.header.stamp = rospy.Time.now()
    vel.twist.linear.x = vX
    vel.twist.linear.y = vY
    vel.twist.linear.z = vZ

    velPub.publish(vel)

def move(msg):
    """
    Function that runs the PID loop for each axis in order to generate a velocity setpoint for each axis

    :param msg: Position target. Used to generate errors in the east-north plane of the UAS. altitudeSetpoint and the
        EKF generates error signals in the 'up' axis of the UAS in order to make the motion as close to planar as
        possible. This is a rospy PoseStamped message. Errors being used are accessed via msg.pose.position.x and
        msg.pose.position.y. This function is defined in the ENU frame for convenience.
    :type msg: float
    :return: void. A velocity setpoint is calculated and published to dr1/velocity_setpoint.
    :rtype: None
    """
    global error, last_error, vel, targetAcquired, landingFlag, error_sum
    if targetAcquired and not landingFlag:
        error[0] = -1.0 * msg.pose.position.x
        error[1] = -1.0 * msg.pose.position.y
        error[2] = altitudeSetpoint - msg.pose.position.z

        error_sum[0] += error[0]
        error_sum[1] += error[1]
        error_sum[2] += error[2]

        velX = (kp * error[0]) + (kd * velocity[0]) + (ki * error_sum[0])
        velY = (kp * error[1]) + (kd * velocity[1]) + (ki * error_sum[1])
        velZ = (kp * error[2]) + (kd * velocity[2]) + (ki * error_sum[2])

        pubVelSetpoint(velX, velY, velZ)
        last_error = error

    if targetAcquired and landingFlag:
        error[0] = -1.0 * msg.pose.position.x
        error[1] = -1.0 * msg.pose.position.y

        error_sum[0] += error[0]
        error_sum[1] += error[1]

        velX = (kp * error[0]) + (kd * velocity[0]) + (ki * error_sum[0])
        velY = (kp * error[1]) + (kd * velocity[1]) + (ki * error_sum[1])

        pubVelSetpoint(velX, velY, descentRate)
        last_error = error

    if not targetAcquired and landingFlag:
        error_sum = numpy.zeros(3)
        pubVelSetpoint(0, 0, descentRate)

    if not targetAcquired and not landingFlag:
        error_sum = numpy.zeros(3)
        pubVelSetpoint(0, 0, 0)


def turn(yaw_degree):
    """
    Update the yaw heading of the UAS

    :param yaw_degree: rotation angle about the "up" axis
    :type yaw_degree: float
    :return: void. Published to dr1/set_pose/orientation.
    """
    yaw_target_pub.publish(yaw_degree)


def set_pose(x, y, z, BODY_FLU=True):
    """
    Method that creates a PoseStamped message for the position publisher.

    :param x: Translation on the 'East' direction in meters
    :type x: float
    :param y: Translation about the 'North' direction in meters
    :type y: float
    :param z: Translation about the 'Up' direction in meters
    :type z: float
    :param BODY_FLU: Boolean describing which frame the message is defined in.
    :return: PoseStamped message containing translations
    :rtype: PoseStamped
    """
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()

    # ROS uses ENU internally, so we will stick to this convention
    if BODY_FLU:
        pose.header.frame_id = 'base_link'

    else:
        pose.header.frame_id = 'map'

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    return pose


def targetAcquisition(msg):
    """
    Update the acquisition state based on th reading from the state estimator. Zero the velocity if the state estimation
    has failed.

    :param msg: Boolean flag. Describes whether SVGS or ArUco have a valid state estimation. This is a rospy boolean. Data accessed via msg.data.
    :type msg: bool
    :return: void. A velocity setpoint is calculated and published to dr1/velocity_setpoint.
    :rtype: None
    """
    global targetAcquired, landingFlag
    targetAcquired = msg.data
    if not targetAcquired and not landingFlag:
        vel.header.stamp = rospy.Time.now()
        vel.twist.linear.x = 0.0
        vel.twist.linear.y = 0.0
        vel.twist.linear.z = 0.0
        velPub.publish(vel)


def landingRoutine(msg):
    """
    If the landing flag is set to true, set the vehicle velocity to zero in the east-north plane, and to the descent
    rate parameter (defined in the associated launch file, depending on the landing technology).

    :param msg: the flag that triggers the landing subroutine. This is a rospy boolean. Data accessed via msg.data.
    :type msg: bool
    :return: void. A velocity setpoint is calculated and published to dr1/velocity_setpoint.
    :rtype: None
    """
    global landingFlag
    landingFlag = msg.data


def localPoseCallback(msg):
    """
    Local callback that updates the internal variable for the vehicle position estimate.

    :param msg: the local pose of the UAS.
    :type msg: rospy.PoseStamped
    :return: void. A velocity setpoint is calculated and published to dr1/velocity_setpoint.
    """
    global localPose
    localPose = msg

def velCallback(msg):
    global velocity
    velocity[0] = msg.twist.linear.y
    velocity[1] = -1.0 * msg.twist.linear.x
    velocity[2] = msg.twist.linear.z

launchTime = rospy.get_param("/launch_time")
launchTime += 10

rospy.init_node("motion_control")
time.sleep(launchTime)
rate = rospy.Rate(freq)

position_target_pub = rospy.Publisher('dr1/set_pose/position', PoseStamped, queue_size=1)
velPub = rospy.Publisher('dr1/velocity_setpoint', TwistStamped, queue_size=1)
yaw_target_pub = rospy.Publisher('dr1/set_pose/orientation', Float32, queue_size=10)
custom_activity_pub = rospy.Publisher('dr1/set_activity/type', String, queue_size=10)

# Long live the SVGS sub.
target_sub = rospy.Subscriber('dr1/target', PoseStamped, move)
ekfPoseSub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, localPoseCallback)
targetAcquisitionSub = rospy.Subscriber('dr1/targetAcquired', Bool, targetAcquisition)
landingFlagSub = rospy.Subscriber('dr1/landing_flag', Bool, landingRoutine)
velSub = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, velCallback)

if __name__ == "__main__":
    rospy.spin()
