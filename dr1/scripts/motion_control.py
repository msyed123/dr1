#!/usr/bin/env python

import time
import numpy
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32, String

freq = 20.0     # hz
dt = 1/freq
error = numpy.zeros(3)
last_error = numpy.zeros(3)
error_sum = numpy.zeros(3)
vel = TwistStamped()

kp = rospy.get_param("/vel_control_kp")
kd = rospy.get_param("/vel_control_kd")
ki = rospy.get_param("/vel_control_ki")


def move(msg):
    global error, last_error, vel
    error[0] = msg.pose.position.x
    error[1] = msg.pose.position.y
    # error[2] = msg.pose.position.z

    error_sum[0] += error[0]
    error_sum[1] += error[1]
    # error_sum[2] += error[2]

    derivative = numpy.zeros(3)
    derivative[0] = (error[0] - last_error[0]) / dt
    derivative[1] = (error[1] - last_error[1]) / dt
    # derivative[2] = (error[2] - last_error[2]) / dt

    velX = (kp * error[0]) + (kd * derivative[0]) + (ki * error_sum[0])
    velY = (kp * error[1]) + (kd * derivative[1]) + (ki * error_sum[1])
    # velZ = (kp * error[2]) + (kd * derivative[2]) + (ki * error_sum[2])

    vel.header.stamp = rospy.Time.now()
    vel.twist.linear.x = velX
    vel.twist.linear.y = velY
    # vel.twist.linear.z = velZ

    velPub.publish(vel)
    last_error = error


def turn(yaw_degree):
    yaw_target_pub.publish(yaw_degree)


# land at current position
def land():
    custom_activity_pub.publish(String("LAND"))


# hover at current position
def hover():
    custom_activity_pub.publish(String("HOVER"))


# return to home position with defined height
def return_home(height):
    position_target_pub.publish(set_pose(0, 0, height, False))


def set_pose(x, y, z, BODY_FLU=True):
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


rospy.init_node("motion_control")
time.sleep(10)
rate = rospy.Rate(20)

position_target_pub = rospy.Publisher('dr1/set_pose/position', PoseStamped, queue_size=1)
velPub = rospy.Publisher('dr1/velocity_setpoint', TwistStamped, queue_size=1)
yaw_target_pub = rospy.Publisher('dr1/set_pose/orientation', Float32, queue_size=10)
custom_activity_pub = rospy.Publisher('dr1/set_activity/type', String, queue_size=10)

# Long live the SVGS sub.
svgs_sub = rospy.Subscriber('dr1/target', PoseStamped, move)

if __name__ == "__main__":
    rospy.spin()
