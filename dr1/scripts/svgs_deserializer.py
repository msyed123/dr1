#!/usr/bin/env python

# from sensor_msgs.msg import Imu, NavSatFix
# from std_msgs.msg import Float32, String, Header
# from pyquaternion import Quaternion
import time

import rospy
# from mavros_msgs.msg import GlobalPositionTarget, State
# from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped


svgs_data = PoseStamped()
x = 0
y = 0
z = 2


def readSerialData():
    # THIS IS PLACEHOLDER AND NEEDS TO BE UPDATED
    global x, y, z, svgs_data
    svgs_data.header.stamp = rospy.Time.now()
    x = 0.1
    y = 0.1
    z = z
    svgs_data.pose.position.x = x
    svgs_data.pose.position.y = y
    svgs_data.pose.position.z = z
    svgs_data.pose.orientation.x = 0.0
    svgs_data.pose.orientation.y = 0.0
    svgs_data.pose.orientation.z = 0.0
    svgs_data.pose.orientation.w = 1.0

    return svgs_data


if __name__ == '__main__':
    try:
        rospy.init_node('svgs_deserializer', anonymous=True)
        svgs_pub = rospy.Publisher('dr1/target', PoseStamped, queue_size=1)
        time.sleep(15)

        while not rospy.is_shutdown():
            target = readSerialData()
            svgs_pub.publish(target)
            print("X: %1.1f\tY:X: %1.1f\tZ: %1.1f, TF: %d", target.pose.position.x, target.pose.position.y,
                  target.pose.position.z, rospy.is_shutdown())
            time.sleep(1/20.0)
            # rate.sleep()

    except rospy.ROSInterruptException:
        pass
