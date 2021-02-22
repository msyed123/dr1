#!/usr/bin/env python

import time
import math
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import OpticalFlowRad
import numpy as np
from filterpy.common import Q_discrete_white_noise, kinematic_kf

launchTime = rospy.get_param("/launch_time")
launchTime += 5.0
freq = 10.0

latestOptFlow = PoseStamped()

f = kinematic_kf(2, 1, dt=(1.0/freq), order_by_dim=False)

def kalmanFilter(msg):
	global latestOptFlow
	latestOptFlow = msg
	x	= latestOptFlow.integrated_x
	y	= latestOptFlow.integrated_y
	z = np.array([x, y])
	f.predict()
	f.update(z)
	# print (f.x)

	positionEstimate = PoseStamped()
	velocityEstimate = TwistStamped()
	positionEstimate.pose.position.x = f.x[0]
	positionEstimate.pose.position.y = f.x[1]
	velocityEstimate.twist.linear.x  = f.x[2]
	velocityEstimate.twist.linear.y  = f.x[3]
	position_pub.publish(positionEstimate)
	velocity_pub.publish(velocityEstimate)

rospy.init_node("kalman_filter")
time.sleep(launchTime)
rate = rospy.Rate(freq)

velocity_pub = rospy.Publisher('dr1/optFlowVelocity', TwistStamped, queue_size=1)
position_pub = rospy.Publisher('dr1/optFlowPosition', PoseStamped, queue_size=1)

SVGS_sub = rospy.Subscriber('mavros/px4flow/raw/optical_flow_rad', OpticalFlowRad, kalmanFilter)

if __name__ == "__main__":
    rospy.spin()

