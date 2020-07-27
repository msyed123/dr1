#!/usr/bin/env python

import time
import math
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32, Int32, Bool

freq = 20.0     # hz
dt = 1/freq

launchTime = rospy.get_param("/launch_time")
launchTime += 5

rospy.init_node("landing_commander")
time.sleep(launchTime)
rate = rospy.Rate(freq)

thresholdTime = rospy.get_param("/threshold_time")

currentTime = time.time()
errorStartTime = time.time()

currentError = float('inf')
currentVelocity = float('inf')

maxError = rospy.get_param("/max_position_error")
maxVelocity = rospy.get_param("/max_velocity_error")

conditionsMet = False
landingFlag = False


def errorCallback(msg):
    global currentError, currentTime, errorStartTime
    currentError = math.sqrt(msg.pose.position.x ** 2 + msg.pose.position.y ** 2)
    currentTime = time.time()
    currentErrorPub.publish(currentError)
    if errorStartTime is None:
        errorStartTime = currentTime


def velocityCallback(msg):
    global currentVelocity
    currentVelocity = math.sqrt(msg.twist.linear.x ** 2 + msg.twist.linear.y ** 2)
    currentVelocityPub.publish(currentVelocity)


def landingCommander(msg):
    global landingFlag, errorStartTime, currentTime, conditionsMet
    if msg.data is not None:
        duration = currentTime - errorStartTime
        if landingFlag is False:
            if currentError < maxError and currentVelocity < maxVelocity:
                if not conditionsMet:
                    errorStartTime = currentTime
                    conditionsMet = True
            else:
                errorStartTime = currentTime
                conditionsMet = False
            if duration >= thresholdTime:
                landingFlag = True
        landingPub.publish(landingFlag)
        counterPub.publish(duration)


landingPub = rospy.Publisher('dr1/landing_flag', Bool, queue_size=1)         # Publisher for the landing flag trigger
counterPub = rospy.Publisher('dr1/landing_counter', Float32, queue_size=1)     # This is for debugging purposes but may become useful
currentErrorPub = rospy.Publisher('dr1/current_error', Float32, queue_size=1)
currentVelocityPub = rospy.Publisher('dr1/current_velocity', Float32, queue_size=1)

errorSub = rospy.Subscriber('dr1/target', PoseStamped, errorCallback)
velocitySub = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, velocityCallback)
triggerSub = rospy.Subscriber('dr1/landing_commander_trigger', Bool, landingCommander)


if __name__ == "__main__":
    rospy.spin()
