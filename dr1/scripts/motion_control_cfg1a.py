#!/usr/bin/env python

import time
import numpy
import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32, String, Bool
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State

freq = 20.0     # hz
dt = 1/freq

origin = numpy.zeros(3)
SVGSPosition = numpy.zeros(3)
error = numpy.zeros(3)
last_error = numpy.zeros(3)
error_sum = numpy.zeros(3)
velocity = numpy.zeros(3)
VIO_velocity = numpy.zeros(3)
VIO_position = numpy.zeros(3)
VIOq = numpy.zeros(4)
vel = TwistStamped()
localPose = PoseStamped()
roll = 0
pitch = 0
yaw = 0
currF = 0
currL = 0
currU = 0
svgsF = 0
svgsL = 0
svgsU = 0
originProcStarted = False
originProcEnded = False
armedBool = False
targetAcquired = False
landingFlag = False
flightMode = ""

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

def move():
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

    # error[0] = origin[0] - VIO_position[0]
    # error[1] = origin[1] - VIO_position[1]
    # error_sum[0] += error[0]
    # error_sum[1] += error[1]
    # velX = (kp * error[0]) + (kd * velocity[0]) + (ki * error_sum[0])
    # velY = (kp * error[1]) + (kd * velocity[1]) + (ki * error_sum[1])
    if landingFlag:
        # pubVelSetpoint(velX, velY, descentRate)
        pubVelSetpoint(0, 0, descentRate)
        # position_target_pub.publish(set_pose(origin[0], origin[1], 0)) # Set position setpoint = XYorigin and Z=0
        # last_error = error
    else:
        # error[2] = altitudeSetpoint - VIO_position[2] # Maybe set velZ = 0 for the first attempts
        # error_sum[2] += error[2]
        # velZ = (kp * error[2]) + (kd * velocity[2]) + (ki * error_sum[2])
        pubVelSetpoint(0, 0, 0)
        # position_target_pub.publish(set_pose(origin[0], origin[1], VIO_position[2])) # Set position setpoint = XYorigin and Z=altitudeSetpoint
        # last_error = error

def turn(yaw_degree):
    """
    Update the yaw heading of the UAS

    :param yaw_degree: rotation angle about the "up" axis
    :type yaw_degree: float
    :return: void. Published to dr1/set_pose/orientation.
    """
    yaw_target_pub.publish(yaw_degree)


def set_pose(x, y, z, BODY_FLU=False):
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
    global localPose, currF, currL, currU
    localPose = msg
    currF = msg.pose.position.x
    currL = msg.pose.position.y
    currU = msg.pose.position.z

def quat2angles(quaternionArray):
	global roll, pitch, yaw
	x = quaternionArray[0]
	y = quaternionArray[1]
	z = quaternionArray[2]
	w = quaternionArray[3]
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + y * y)
	roll = math.atan2(t0, t1)
	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	pitch = math.asin(t2)
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	yaw = math.atan2(t3, t4)

def SVGS2FLU(xyzSVGS):
    global svgsF, svgsL, svgsU
    xSVGS = xyzSVGS[0]
    ySVGS = xyzSVGS[1]
    zSVGS = xyzSVGS[2]
    svgsF = ySVGS*math.cos(-1.0*yaw) - xSVGS*math.sin(-1.0*yaw)
    svgsL = -1.0 * ySVGS*math.sin(-1.0*yaw) - xSVGS*math.cos(-1.0*yaw)
    svgsU = zSVGS
    svgsFLU = PoseStamped()
    svgsFLU.pose.position.x = svgsF
    svgsFLU.pose.position.y = svgsL
    svgsFLU.pose.position.z = svgsU
    svgsFLU_pub.publish(svgsFLU)

def velCallback(msg):
    global velocity
    velocity[0] = msg.twist.linear.y
    velocity[1] = msg.twist.linear.z
    velocity[2] = msg.twist.linear.x

def vioData(msg):
    global VIO_velocity, VIO_position, VIO_t
    VIO_t = msg.header.stamp
    VIO_position[0] = msg.pose.pose.position.y
    VIO_position[1] = -1.0 * msg.pose.pose.position.x
    VIO_position[2] = -1.0 * msg.pose.pose.position.z
    VIO_velocity[0] = msg.twist.twist.linear.y
    VIO_velocity[1] = -1.0 * msg.twist.twist.linear.x
    VIO_velocity[2] = -1.0 * msg.twist.twist.linear.z
    VIOq[0] = msg.pose.pose.orientation.x
    VIOq[1] = msg.pose.pose.orientation.y
    VIOq[2] = msg.pose.pose.orientation.z
    VIOq[3] = msg.pose.pose.orientation.w
    quat2angles(VIOq)
    # print("yaw: %6.3f\t| pitch: %6.3f\t| roll: %6.3f" % (yaw,pitch,roll))

def modeData(msg):
    global armedBool, flightMode
    armedBool = msg.armed
    flightMode = msg.mode

def findOrigin():
    global originProcStarted, originProcEnded, origin, vel
    originProcStarted = True
    localCounter = 0
    print("This should be printed only once")
    rospy.loginfo("This should be logged only once")
    start_time = time.time()
    while time.time()-start_time < 5:
        vel.header.stamp = rospy.Time.now()
        vel.twist.linear.x = 0.0
        vel.twist.linear.y = 0.0
        vel.twist.linear.z = 0.0
        velPub.publish(vel)
        # position_target_pub.publish(set_pose(VIO_position[0], VIO_position[1], VIO_position[2])) # Set position setpoint = currLnt VIO position
        if SVGSPosition[0] != 0 and SVGSPosition[1] != 0 and SVGSPosition[2] != 0:
            # origin[0] += VIO_position[0] + SVGSPosition[0]
            # origin[1] += VIO_position[1] - SVGSPosition[1]
            SVGS2FLU(SVGSPosition)
            origin[0] += currF + svgsF#(currF + svgsF)*math.cos(-1.0*yaw) + (currL + svgsL)*math.sin(-1.0*yaw)
            origin[1] += currL + svgsL#(currL + svgsL)*math.cos(-1.0*yaw) - (currF + svgsF)*math.sin(-1.0*yaw)
            localCounter += 1
        time.sleep(0.05)
    print("Datapoints for the origin: " + str(localCounter))
    # rospy.loginfo("Datapoints for the origin: %i", localCounter)
    origin[0] = origin[0]/localCounter
    origin[1] = origin[1]/localCounter
    origin_location = PoseStamped()
    origin_location.pose.position.x = origin[0]
    origin_location.pose.position.y = origin[1]
    origin_location.pose.position.z = 0
    origin_pub.publish(origin_location)
    originProcEnded = True
    origin_found_pub.publish(True)

def SVGScallback(msg):
    global SVGSPosition, vel, VIO_position
    SVGSPosition[0] = msg.pose.position.x
    SVGSPosition[1] = msg.pose.position.y
    SVGSPosition[2] = msg.pose.position.z
    # if SVGSPosition[0] != 0 and SVGSPosition[1] != 0 and SVGSPosition[2] !=0:
    #     SVGS2FLU(SVGSPosition)
    #     origin[0] = currF + svgsF#(currF + svgsF)*math.cos(-1.0*yaw) + (currL + svgsL)*math.sin(-1.0*yaw)
    #     origin[1] = currL + svgsL#(currL + svgsL)*math.cos(-1.0*yaw) - (currF + svgsF)*math.sin(-1.0*yaw)
    #     origin_location = PoseStamped()
    #     origin_location.pose.position.x = origin[0]
    #     origin_location.pose.position.y = origin[1]
    #     origin_location.pose.position.z = 0
    #     origin_pub.publish(origin_location)
    if flightMode == "OFFBOARD" and originProcStarted is False: #This case should trigger immediatly after switching to offboard - Should only run once
        findOrigin()
    if flightMode == "OFFBOARD" and originProcEnded is True:
        move()
    else: # This case is triggered when the origin finding process is occurring and when flying towards the target. Used so that landingCommander is not triggered
        origin_found_pub.publish(False)
        vel.twist.linear.x = 0.0
        vel.twist.linear.y = 0.0
        vel.twist.linear.z = 0.0
        velPub.publish(vel)
        position_target_pub.publish(set_pose(VIO_position[0], VIO_position[1], VIO_position[2]))


# def mavrosPoseData(msg):
    # print("MAV X: %6.3f\t| MAV Y: %6.3f\t| MAV Z: %6.3f" % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
    # /mavros/local_position/pose/pose/position/x

launchTime = rospy.get_param("/launch_time")
launchTime += 10
rospy.init_node("motion_control")
time.sleep(launchTime)
rate = rospy.Rate(freq)

position_target_pub = rospy.Publisher('dr1/set_pose/position', PoseStamped, queue_size=1)
velPub = rospy.Publisher('dr1/velocity_setpoint', TwistStamped, queue_size=1)
yaw_target_pub = rospy.Publisher('dr1/set_pose/orientation', Float32, queue_size=10)
custom_activity_pub = rospy.Publisher('dr1/set_activity/type', String, queue_size=10)
origin_found_pub = rospy.Publisher('dr1/origin_found', Bool, queue_size=1)
origin_pub = rospy.Publisher('dr1/origin_position', PoseStamped, queue_size=1)
svgsFLU_pub = rospy.Publisher('dr1/svgsFLU',PoseStamped, queue_size=1)

# Long live the SVGS sub.
target_sub = rospy.Subscriber('dr1/target', PoseStamped, SVGScallback)
armedSub = rospy.Subscriber('/mavros/state', State, modeData)
ekfPoseSub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, localPoseCallback)
targetAcquisitionSub = rospy.Subscriber('dr1/targetAcquired', Bool, targetAcquisition)
landingFlagSub = rospy.Subscriber('dr1/landing_flag', Bool, landingRoutine)
velSub = rospy.Subscriber('/dr1/kfVelocity', TwistStamped, velCallback)
vioSub = rospy.Subscriber('/camera/odom/sample_throttled', Odometry, vioData)
# mavPoseSub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, mavrosPoseData)

if __name__ == "__main__":
    rospy.spin()
