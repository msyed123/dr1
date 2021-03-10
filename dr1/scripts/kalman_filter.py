#!/usr/bin/env python

import time
import math
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
from filterpy.common import Q_discrete_white_noise, kinematic_kf

launchTime = rospy.get_param("/launch_time")
launchTime += 5.0
freq = 20.0

latestSVGS = PoseStamped()

f = kinematic_kf(3, 1, dt=(1.0/freq), order_by_dim=False)

def kalmanFilter(msg):
	global latestSVGS, latestVel, latestAcc, t0, t1, f
	latestSVGS = msg
	x	= latestSVGS.pose.position.y
	y	= latestSVGS.pose.position.z
	z	= latestSVGS.pose.position.x
	z = np.array([x, y, z])
	f.predict()
	f.update(z)
	# print (f.x)

	positionEstimate = PoseStamped()
	velocityEstimate = TwistStamped()
	positionEstimate.pose.position.x = f.x[0]
	positionEstimate.pose.position.y = f.x[1]
	positionEstimate.pose.position.z = f.x[2]
	velocityEstimate.twist.linear.x  = f.x[3]
	velocityEstimate.twist.linear.y  = f.x[4]
	velocityEstimate.twist.linear.z  = f.x[5]
	position_pub.publish(positionEstimate)
	velocity_pub.publish(velocityEstimate)

rospy.init_node("kalman_filter")
time.sleep(launchTime)
rate = rospy.Rate(freq)

velocity_pub = rospy.Publisher('dr1/kfVelocity', TwistStamped, queue_size=1)
position_pub = rospy.Publisher('dr1/kfPosition', PoseStamped, queue_size=1)

SVGS_sub = rospy.Subscriber('dr1/target', PoseStamped, kalmanFilter)

if __name__ == "__main__":
    rospy.spin()

