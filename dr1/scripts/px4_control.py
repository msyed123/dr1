#!/usr/bin/env python

import math
import time
import numpy

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from pyquaternion import Quaternion
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String

imu = None
gps = None
local_pose = None
current_state = None
current_heading = None
local_enu_position = None
mavros_state = None
cur_target_pose = None
global_target = None
state = None
takeoff_height = 2
posSetpoint = numpy.zeros(3)
velSetpoint = numpy.zeros(3)
originFLU = numpy.zeros(3)
received_new_task = False
arm_state = False
offboard_state = False
received_imu = False
frame = "BODY"
mode = ""

'''
ros services
'''
armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)

'''
ros publishers
'''
# local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
local_target_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)

def start():
    """
    :rtype: void
    :return: void type.
    """
    global cur_target_pose, arm_state, offboard_state, state
    rospy.init_node("offboard_node")
    launchTime = rospy.get_param("/launch_time")
    time.sleep(launchTime)

    for i in range(10):
        if current_heading is not None:
            break
        else:
            print("Waiting for initialization.")
            time.sleep(1)
    # cur_target_pose = construct_target(0, 0, takeoff_height, current_heading)
    #
    # # Takeoff loop
    # for i in range(10):
    #     local_target_pub.publish(cur_target_pose)
    #     arm_state = arm()
    #     offboard_state = offboard()
    #     time.sleep(0.2)

    '''
    main ROS thread
    '''
    while arm_state and offboard_state and (rospy.is_shutdown() is False):
        # while rospy.is_shutdown() is False:
        # local_target_pub.publish(cur_target_pose)
        if (state is "LAND") and (local_pose.pose.position.z < 0.1):
            if disarm():
                state = "DISARMED"
        time.sleep(0.1)

def construct_target(x, y, z, vx, vy, vz):
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()
    target_raw_pose.coordinate_frame = 1 # MAV_FRAME_LOCAL_NED
    target_raw_pose.position.x = x
    target_raw_pose.position.y = y
    target_raw_pose.position.z = z
    target_raw_pose.velocity.x = vx
    target_raw_pose.velocity.y = vy
    target_raw_pose.velocity.z = vz
    target_raw_pose.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.FORCE + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE
    # target_raw_pose.yaw = yaw
    # target_raw_pose.yaw_rate = yaw_rate
    return target_raw_pose

def position_distance(cur_p, target_p, threshold=0.1):
    delta_x = math.fabs(cur_p.pose.position.x - target_p.position.x)
    delta_y = math.fabs(cur_p.pose.position.y - target_p.position.y)
    delta_z = math.fabs(cur_p.pose.position.z - target_p.position.z)

    if (math.sqrt((delta_x ** 2) + (delta_y ** 2) + (delta_z ** 2))) < threshold:
        return True
    else:
        return False

def local_pose_callback(msg):
    global local_pose, local_enu_position
    local_pose = msg
    local_enu_position = msg

def imu_callback(msg):
    global imu, current_heading, received_imu
    imu = msg
    current_heading = q2yaw(imu.orientation)
    received_imu = True

def gps_callback(msg):
    global gps
    gps = msg

def FLU2ENU(msg):
    FLU_x = msg.pose.position.x * math.cos(current_heading) - msg.pose.position.y * math.sin(current_heading)
    FLU_y = msg.pose.position.x * math.sin(current_heading) + msg.pose.position.y * math.cos(current_heading)
    FLU_z = msg.pose.position.z
    return FLU_x, FLU_y, FLU_z

def set_target_position_callback(msg):
    global frame, cur_target_pose, posSetpoint
    posSetpoint[0] = msg.pose.position.x
    posSetpoint[1] = msg.pose.position.y
    posSetpoint[2] = msg.pose.position.z
    # if msg.header.frame_id == 'base_link':
    #     '''
    #     BODY_FLU
    #     '''
    #     # For Body frame, we will use FLU (Forward, Left and Up)
    #     #           +Z     +X
    #     #            ^    ^
    #     #            |  /
    #     #            |/
    #     #  +Y <------body
    #     frame = "BODY"
    #     print("body FLU frame")
    #     ENU_X, ENU_Y, ENU_Z = FLU2ENU(msg)
    #     ENU_X = ENU_X + local_pose.pose.position.x
    #     ENU_Y = ENU_Y + local_pose.pose.position.y
    #     ENU_Z = ENU_Z + local_pose.pose.position.z
    #     cur_target_pose = construct_target(ENU_X, ENU_Y, ENU_Z, current_heading)
    # else:
    #     '''
    #     LOCAL_ENU
    #     '''
    #     # For world frame, we will use ENU (EAST, NORTH and UP)
    #     #     +Z     +Y
    #     #      ^    ^
    #     #      |  /
    #     #      |/
    #     #    world------> +X
    #     frame = "LOCAL_ENU"
    #     print("local ENU frame")
    #     cur_target_pose = construct_target(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, current_heading)

def velocity_setpoint_callback(msg):
    global velSetpoint
    velSetpoint[0] = msg.twist.linear.x
    velSetpoint[1] = msg.twist.linear.y
    velSetpoint[2] = msg.twist.linear.z
    # if msg.header.frame_id is "enu":
    #     velocitySetpoint = TwistStamped()
    #     velocitySetpoint.twist.linear.x = msg.twist.linear.y
    #     velocitySetpoint.twist.linear.y = msg.twist.linear.x
    #     velocitySetpoint.twist.linear.z = -1.0 * msg.twist.linear.z
    #     local_vel_pub.publish(velocitySetpoint)
    # else:
    local_vel_pub.publish(msg)

def custom_activity_callback(msg):
    global cur_target_pose, state
    print("Received Custom Activity:", msg.data)

    if msg.data == "LAND":
        print("LANDING!")
        state = "LAND"
        cur_target_pose = construct_target(local_pose.pose.position.x,
                                           local_pose.pose.position.y,
                                           0.1,
                                           current_heading)

    if msg.data == "HOVER":
        print("HOVERING!")
        state = "HOVER"
        hover()

    else:
        print("Received Custom Activity: ", msg.data, " -> not supported.")


def set_target_yaw_callback(msg):
    global cur_target_pose
    print("Received New Yaw Task!")

    yaw_deg = msg.data * math.pi / 180.0
    cur_target_pose = construct_target(local_pose.pose.position.x,
                                       local_pose.pose.position.y,
                                       local_pose.pose.position.z,
                                       yaw_deg)


def q2yaw(q):
    if isinstance(q, Quaternion):
        rotate_z_rad = q.yaw_pitch_roll[0]
    else:
        q_ = Quaternion(q.w, q.x, q.y, q.z)
        rotate_z_rad = q_.yaw_pitch_roll[0]

    return rotate_z_rad


def arm():
    if armService(True):
        return True
    else:
        print("Vehicle arming failed!")
        return False


def disarm():
    if armService(False):
        return True
    else:
        print("Vehicle disarming failed!")
        return False


def offboard():
    if flightModeService(custom_mode='OFFBOARD'):
        return True
    else:
        print("Vehicle Offboard failed")
        return False


# noinspection PyUnresolvedReferences
def hover():
    global cur_target_pose, local_pose
    cur_target_pose = construct_target(local_pose.pose.position.x, local_pose.pose.position.y,
                                       local_pose.pose.position.z, current_heading)


# noinspection PyUnresolvedReferences
def takeoff_detection():
    global local_pose, offboard_state, arm_state
    if local_pose.pose.position.z > 0.1 and offboard_state and arm_state:
        return True
    else:
        return False

def flightMode(msg):
    global mode
    mode = msg.mode

def originData(msg):
    global originFLU
    originFLU[0] = msg.pose.position.x
    originFLU[1] = msg.pose.position.y
    originFLU[2] = 0

def targetSub(msg):
    global originFLU
    originTarget = PoseStamped()
    originTarget.pose.position.x = originFLU[0]
    originTarget.pose.position.y = originFLU[1]
    originTarget.pose.position.z = 0
    local_target_pub.publish(originTarget)
    print("Command to origin sent: x =  " + str(originFLU[0]) + " | y = " + str(originFLU[1]) + " | z = " + str(originFLU[2]))

'''
ros subscribers
'''
local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_pose_callback)
gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)
imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, imu_callback)

set_target_position_sub = rospy.Subscriber("dr1/set_pose/position", PoseStamped, set_target_position_callback)
velocity_setpoint_sub = rospy.Subscriber("dr1/velocity_setpoint", TwistStamped, velocity_setpoint_callback)
set_target_yaw_sub = rospy.Subscriber("dr1/set_pose/orientation", Float32, set_target_yaw_callback)
custom_activity_sub = rospy.Subscriber("dr1/set_activity/type", String, custom_activity_callback)
target_sub = rospy.Subscriber("dr1/target", PoseStamped, targetSub)
origin_sub = rospy.Subscriber("dr1/origin_position", PoseStamped, originData)
modeSub = rospy.Subscriber('/mavros/state', State, flightMode)

if __name__ == '__main__':
    start()
    rospy.spin()
