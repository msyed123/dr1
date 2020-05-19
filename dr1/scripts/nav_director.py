#!/usr/bin/env python

import rospy
# from mavros_msgs.msg import GlobalPositionTarget, State
# from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist, Point
# from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String, Header
# from pyquaternion import Quaternion
import time

updateFreq = 20.0   # Hertz


class Controller:
    def __init__(self, maxX, maxY, maxZ):
        self.target = PoseStamped()
        self.currentLocation = Point()

        self.currentLocation.x = 0
        self.currentLocation.y = 0
        self.currentLocation.z = 0

        self.maxXVel = maxX
        self.maxYVel = maxY
        self.maxZVel = maxZ

        self.setpointPub = rospy.Publisher('dr1/set_pose/position', PoseStamped, queue_size=1)
        self.rate = rospy.Rate(updateFreq)
        self.yaw_target_pub = rospy.Publisher('dr1/set_pose/orientation', Float32, queue_size=10)
        self.custom_activity_pub = rospy.Publisher('dr1/set_activity/type', String, queue_size=10)

    def updateLocation(self, msg):
        self.currentLocation.x = msg.x
        self.currentLocation.y = msg.y
        self.currentLocation.z = msg.z

    def calcStep(self):
        # TODO: Incorporate a series PID loop with both the X and Y axes
        targetX = (self.currentLocation.x / self.maxXVel) / updateFreq   # The position step is the distance to the target divided by the max velocity times the time step
        targetY = (self.currentLocation.y / self.maxYVel) / updateFreq
        targetZ = (self.currentLocation.z / self.maxZVel) / updateFreq

        self.move(targetX, targetY, targetZ, True)
        self.rate.sleep()

    def move(self, x, y, z, BODY_OFFSET_ENU=True):
        self.setpointPub.publish(self.set_pose(x, y, z, BODY_OFFSET_ENU))

    def turn(self, yaw_degree):
        self.yaw_target_pub.publish(yaw_degree)

        # land at current position
    def land(self):
        self.custom_activity_pub.publish(String("LAND"))

        # hover at current position
    def hover(self):
        self.custom_activity_pub.publish(String("HOVER"))

        # return to home position with defined height
    def return_home(self, height):
        self.position_target_pub.publish(self.set_pose(0, 0, height, False))

    def set_pose(self, x, y, z, BODY_FLU):
        pose = PoseStamped()
        pose.header = Header()
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


if __name__ == '__main__':
    rospy.init_node('nav_director', anonymous=True)
    con = Controller(2, 2, 1)
    time.sleep(2)
    svgsSub = rospy.Subscriber('dr1/svgs_data/relative_position', Point, con.updateLocation)

    while not rospy.is_shutdown():
        con.calcStep()
