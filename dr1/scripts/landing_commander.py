#!/usr/bin/env python

import time
import math
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32, Int32, Bool
from mavros_msgs.msg import State

freq = 20.0     # hz
maxError = rospy.get_param("/max_position_error")
maxVelocity = rospy.get_param("/max_velocity_error")
thresholdTime = rospy.get_param("/threshold_time")
launchTime = rospy.get_param("/launch_time")
rospy.init_node("landing_commander")
launchTime += 5
time.sleep(launchTime)
rate = rospy.Rate(freq)
currentTime = time.time()
last_t = currentTime
errorStartTime = currentTime
dt = 0
currentError = float('inf')
currentVelocity = float('inf')
duration = 0
conditionsMet = False
originFound = False
landingFlag = False

print("Landing commander node active")

def errorCallback(msg):     # /dr1/target
    global currentError, currentTime, errorStartTime, last_t, dt
    currentError = math.sqrt(msg.pose.position.x ** 2 + msg.pose.position.y ** 2)
    dt = time.time() - last_t
    last_t = time.time()
    currentErrorPub.publish(currentError)

def velocityCallback(msg):  # /mavros/local_position/velocity_body
    global currentVelocity
    currentVelocity = math.sqrt(msg.twist.linear.x ** 2 + msg.twist.linear.y ** 2)
    currentVelocityPub.publish(currentVelocity)

def flightMode(msg):        # /mavros/state
    global mode
    mode = msg.mode

def originFoundSub(msg):
    global originFound
    originFound = msg.data

def landingCommander(msg):  # /dr1/landing_commander_trigger
    global landingFlag, errorStartTime, currentTime, conditionsMet, duration
    if landingFlag is False and mode == 'OFFBOARD' and originFound:
        if msg.data and dt <= 0.5 and currentError < maxError and currentVelocity < maxVelocity and errorStartTime is not None:
            duration = time.time() - errorStartTime
            if duration >= thresholdTime:
                landingFlag = True
        else:
            # This can only be triggered if the the mode is offboard and if the landingFlag is still false
            # This else statement will reset the counter if SVGS returns 3 successive 0's (svgs_deserializer.py), or if the dt between SVGS packets published is > 0.5s
            # or if the position/velocity errors are greater than the max values
            duration = 0.0
            errorStartTime = time.time()
    elif landingFlag is True:
        pass
    else:
        # This else statement will reset the counter if the mode isn't OFFBOARD, or if the landingFlag is True
        duration = 0.0
        errorStartTime = time.time()

    landingPub.publish(landingFlag)
    counterPub.publish(duration)


landingPub = rospy.Publisher('dr1/landing_flag', Bool, queue_size=1)            # Publisher for the landing flag trigger
counterPub = rospy.Publisher('dr1/landing_counter', Float32, queue_size=1)      # This is for debugging purposes but may become useful
currentErrorPub = rospy.Publisher('dr1/current_error', Float32, queue_size=1)
currentVelocityPub = rospy.Publisher('dr1/current_velocity', Float32, queue_size=1)

errorSub = rospy.Subscriber('dr1/target', PoseStamped, errorCallback)
velocitySub = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, velocityCallback)
triggerSub = rospy.Subscriber('dr1/landing_commander_trigger', Bool, landingCommander)
modeSub = rospy.Subscriber('/mavros/state', State, flightMode)
originSub = rospy.Subscriber('dr1/origin_found', Bool, originFoundSub)

if __name__ == "__main__":
    rospy.spin()
