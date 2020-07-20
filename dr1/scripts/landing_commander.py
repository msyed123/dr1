#!/usr/bin/env python

import time
import math
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32, String, Bool

freq = 20.0     # hz
dt = 1/freq

thresholdTime = rospy.get_param("/threshold_time")
activationCount = thresholdTime * freq
counter = 0

currentError = float('inf')
currentVelocity = float('inf')

maxError = rospy.get_param("/max_position_error")
maxVelocity = rospy.get_param("/max_velocity_error")

landingFlag = False

landingPub = rospy.Publisher('dr1/landingFlag', Bool, queue_size=1)

while not landingFlag:
    if currentError < maxError and currentVelocity and maxVelocity:
        counter += 1
        time.sleep(dt)
    else:
        counter = 0
    landingPub.publish(landingFlag)
    if counter > activationCount:
        landingFlag = True

while landingFlag:
    landingPub.publish(landingFlag)
    time.sleep(dt)


def errorCallback(msg):
    global currentError
    currentError = math.sqrt(msg.pose.position.x ** 2 + msg.pose.position.y ** 2)


def velocityCallback(msg):
    global currentVelocity
    currentVelocity = math.sqrt(msg.twist.linear.x ** 2 + msg.twist.linear.y ** 2)


errorSub = rospy.Subscriber('dr1/target', PoseStamped, errorCallback)
velocityPub = rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, velocityCallback)
