#!/usr/bin/env python

import time
import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool

freq = 20.0  # hz
dt = 1 / freq

currentError = float('inf')
currentVelocity = float('inf')

thresholdTime = 5.0  # seconds
maxError = 0.05  # meters
maxVelocity = 0.5  # meters/sec

activationCount = freq * thresholdTime
count = 0

landFlag = False

while not landFlag:
    global landingFlagPub
    if currentError < maxError and currentVelocity < maxVelocity:
        count += 1
        time.sleep(dt)
    else:
        count = 0
    landingFlagPub.publish(landFlag)
    if count > activationCount:
        landFlag = True

while landFlag:
    global landingFlagPub
    landingFlagPub.publish(landFlag)
    time.sleep(dt)


def errorCallback(msg):
    global currentError
    currentError = math.sqrt(msg.pose.position.x ** 2 + msg.pose.position.y ** 2)


def velocityCallback(msg):
    global currentVelocity
    currentVelocity = math.sqrt(msg.twist.linear.x ** 2 + msg.twist.linear.y ** 2)


rospy.init_node("landing_commander")
time.sleep(5)
rate = rospy.Rate(freq)

target_sub = rospy.Subscriber('dr1/target', PoseStamped, errorCallback)
velocity_sub = rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, velocityCallback)

landingFlagPub = rospy.Publisher('dr1/landingFlag', Bool, queue_size=1)

if __name__ == "__main__":
    rospy.spin()
