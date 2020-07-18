#!/usr/bin/env python

import rospy
# from mavros_msgs.msg import GlobalPositionTarget, State
# from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist, Point
# from sensor_msgs.msg import Imu, NavSatFix
# from std_msgs.msg import Float32, String, Header
# from pyquaternion import Quaternion
import time


class Controller:
    def __init__(self):
        """
        Set up place holder data for controller testing and PID calibration for velocity control nodes.
        
        :type Controller: Intializing svgs
        """
        self.svgs_data = Point()
        self.svgs_data.x = 0
        self.svgs_data.y = 0
        self.svgs_data.z = 2

    def readSerialData(self):
        """
        Update placeholder data with a simulated deserialization of SVGS data.
        
        :return : Creates a valid message to send over ROS with the help of placeholder
        """
        # THIS IS PLACEHOLDER AND NEEDS TO BE UPDATED
        self.svgs_data.x += 1
        self.svgs_data.y += 1
        self.svgs_data.z += 0

        return self.svgs_data


def main():
    cnt = Controller()
    time.sleep(2)
    rospy.init_node('svgs_deserializer', anonymous=True)
    svgs_pub = rospy.Publisher('dr1/svgs_data/relative_position', Point, queue_size=1)
    rate = rospy.Rate(20.0)
    cnt.readSerialData()

    while not rospy.is_shutdown():
        svgs_pub.publish(cnt.readSerialData())
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
