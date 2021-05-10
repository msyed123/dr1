#!/usr/bin/env python

import os
import os.path
from os import listdir, system
import time
import datetime
import numpy
import rospy
import csv
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32, String, Bool
from nav_msgs.msg import Odometry

armedBool = False
currError = 0.0
currVel = 0.0
counter = 0.0
landFlag = False
targetAcq = False

def mavrosData(msg):
	global armedBool, mode
	armedBool = msg.armed
	mode = msg.mode

def vSetPointData(msg):
	global vSetPoint
	vSetPoint[0] = msg.twist.linear.x
	vSetPoint[1] = msg.twist.linear.y
	vSetPoint[2] = msg.twist.linear.z

def SVGSData(msg):
	global SVGSPosition
	SVGSPosition[0] = msg.pose.position.x
	SVGSPosition[1] = msg.pose.position.y
	SVGSPosition[2] = msg.pose.position.z

def targetAcquiredData(msg):
	global targetAcq
	targetAcq = msg.data
	
def landFlagData(msg):
	global landFlag
	landFlag = msg.data

def counterData(msg):
	global counter
	counter = msg.data

def kfPosData(msg):
	global kfPosition
	kfPosition[0] = msg.pose.position.x
	kfPosition[1] = msg.pose.position.y
	kfPosition[2] = msg.pose.position.z

def kfVelData(msg):
	global kfPosition
	kfVelocity[0] = msg.twist.linear.x
	kfVelocity[1] = msg.twist.linear.y
	kfVelocity[2] = msg.twist.linear.z

def currErrorData(msg):
	global currError
	currError = msg.data

def currVelData(msg):
	global currVel
	currVel = msg.data

def velocityData(msg):
	global velocity, MAVROS_t
	MAVROS_t = msg.header.stamp
	velocity[0] = msg.twist.linear.x
	velocity[1] = msg.twist.linear.y
	velocity[2] = msg.twist.linear.z

# Uncomment this block for HIL testing
# if armedBool:
# 	with open(path, mode='a') as csv_file:
# 		CSV_write = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
# 		CSV_write.writerow([MAVROS_t, 
# 							SVGSPosition[0], SVGSPosition[1], SVGSPosition[2],
# 							kfPosition[0], kfPosition[1], kfPosition[2],
#							"VIO_position[0]", "VIO_position[1]", "VIO_position[2]",
# 							"VIO_velocity[0]", "VIO_velocity[1]", "VIO_velocity[2]",
# 							kfVelocity[0], kfVelocity[1], kfVelocity[2],
# 							velocity[0],velocity[1],velocity[2],
# 							vSetPoint[0], vSetPoint[1], vSetPoint[2],
# 							currError,
# 							currVel,
# 							mode,
# 							armedBool,
# 							targetAcq,
# 							landFlag,
# 							counter])

# Comment this block for HIL testing.
def vioData(msg):
	global VIO_velocity, VIO_position, VIO_t
	VIO_t = msg.header.stamp
	VIO_position[0] = msg.pose.pose.position.y
	VIO_position[1] = msg.pose.pose.position.x
	VIO_position[2] = -1.0 * msg.pose.pose.position.z
	VIO_velocity[0] = msg.twist.twist.linear.y
	VIO_velocity[1] = msg.twist.twist.linear.x
	VIO_velocity[2] = -1.0 * msg.twist.twist.linear.z
	if armedBool:
		with open(path, mode='a') as csv_file:
			CSV_write = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
			CSV_write.writerow([VIO_t,SVGSPosition[0],
			SVGSPosition[1], SVGSPosition[2],
			kfPosition[0], kfPosition[1], kfPosition[2],
			VIO_position[0], VIO_position[1], VIO_position[2],
			VIO_velocity[0], VIO_velocity[1], VIO_velocity[2],
			kfVelocity[0], kfVelocity[1], kfVelocity[2],
			velocity[0],velocity[1],velocity[2],
			vSetPoint[0], vSetPoint[1], vSetPoint[2],
			currError,
			currVel,
			mode,
			armedBool,
			targetAcq,
			landFlag,
			counter])

VIO_velocity = numpy.zeros(3)
VIO_position = numpy.zeros(3)
vSetPoint = numpy.zeros(3)
SVGSPosition = numpy.zeros(3)
kfPosition = numpy.zeros(3)
kfVelocity = numpy.zeros(3)
velocity = numpy.zeros(3)

launchTime = rospy.get_param("/launch_time")
launchTime += 5
freq = 20.0
dt = 1/freq

rospy.init_node("data_recording")
time.sleep(launchTime)
rate = rospy.Rate(freq)

current_date = str(datetime.datetime.now()).split(" ")[0]
dataDir = "/home/dev/Desktop/DataLogs"
if not os.path.exists(dataDir):
    os.mkdir(dataDir)
currentDateDir = dataDir + "/" + current_date
if not os.path.exists(currentDateDir):
    os.mkdir(currentDateDir)
fileCount = len(os.listdir(currentDateDir))
path = currentDateDir + "/" + current_date + "_" + str(fileCount+1) + ".csv"

with open(path, mode='w') as csv_file:
    CSV_write = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    CSV_write.writerow(["Time",
						"SVGS X","SVGS Y","SVGS Z",
						"KF X","KF Y","KF Z",
						"VIO X", "VIO Y", "VIO Z",
						"VIO Vx","VIO Vy","VIO Vz",
						"KF Vx","KF Vy","KF Vz",
						"Vx","Vy","Vz",
						"Setpoint Vx","Setpoint Vy","Setpoint Vz",
						"Current Error",
						"Current Velocity",
						"Flight Mode",
						"Armed Bool",
						"Target Acquired",
						"Landing Flag",
						"Landing Counter"])

rospy.loginfo("Recording logfile to: %s", path)

velocitySetpointSub = rospy.Subscriber('/dr1/velocity_setpoint', TwistStamped, vSetPointData)
targetSub = rospy.Subscriber('dr1/target', PoseStamped, SVGSData)
targetAcquiredSub = rospy.Subscriber('dr1/targetAcquired', Bool, targetAcquiredData)
landFlagSub = rospy.Subscriber('dr1/landing_flag', Bool, landFlagData)
counterSub = rospy.Subscriber('dr1/landing_counter', Float32, counterData)
armedSub = rospy.Subscriber('/mavros/state', State, mavrosData)
kfPosSub = rospy.Subscriber('dr1/kfPosition', PoseStamped, kfPosData)
kfVelSub = rospy.Subscriber('dr1/kfVelocity', TwistStamped, kfVelData)
currErrorSub = rospy.Subscriber('dr1/current_error', Float32, currErrorData)
currVelSub = rospy.Subscriber('dr1/current_velocity', Float32, currVelData)
velocitySub = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, velocityData)
vioSub = rospy.Subscriber('/camera/odom/sample_throttled', Odometry, vioData)

if __name__ == "__main__":
    rospy.spin()
