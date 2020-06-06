#!/usr/bin/env python

import rospy. """ Writing a ROS Node """
from mavros_msgs.msg import GlobalPositionTarget, State """ Using the global position information fused by FCU and raw GPS data """
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode 
from geometry_msgs.msg import PoseStamped, Twist, Point
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion
import time
import math


class Commander:
    def __init__(self): 
        rospy.init_node("motion_controller") #Initializing the ROS node for the process!motion_controller is the name of the node
             
        self.rate = rospy.Rate(20) #Attempt to keep the loop at 20 Hz rate

        # Class variables
        self.currentPose = PoseStamped() # Posestamped is a Pose with reference coordinate frame and timestamp
        self.target = PoseStamped()

        # Publishers: These are the setpoints that the flight computers uses to update the setpoints for position, yaw
        # and any specially defined activities that trigger a subroutine
        self.position_target_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.yaw_target_pub = rospy.Publisher('mavros/set_pose/orientation', Float32, queue_size=10)
        self.custom_activity_pub = rospy.Publisher('mavros/set_activity/type', String, queue_size=10)

        # Subscribers: These are messages that require a callback that does some state mutation with respect to the
        # received message. In our case this will be an update to a setpoint that the flight computer will convert
        # to a position update based on the position estimates from the onboard EKFs
        self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.update_pose)
        self.svgs_sub = rospy.Subscriber("dr1/relative_position", Point, self.svgs_motion)

        # Initialization functions
        self.update_pose()
        self.spoolUp()

    def set_pose(self, x, y, z, BODY_FLU=True):
        """
        This is the method that creates the PoseStamped setpoints for the ROS topics that update the position setpoint
        :param x: The shift in the forward direction of the vehicle (forward is positive, FLU frame definition)
        :param y: The shift in the sideways direction of the vehicle (left is positive, FLU frame definition)
        :param z: The shift in the vertical axis of the vehicle (up is positive, FLU frame definition)
        :param BODY_FLU: The frame definition of the vehicle. Either FLU or ENU. Currently only FLU works. Sorry.
        :return: Returns a target setpoint of type PoseStamped to be used with the position target publisher
        """
        self.target.header.stamp = rospy.Time.now()

        # ROS uses ENU internally, so we will stick to this convention
        if BODY_FLU:
            self.target.header.frame_id = 'base_link'

        else: 
            self.target.header.frame_id = 'map'

        self.target.pose.position.x = x
        self.target.pose.position.y = y
        self.target.pose.position.z = z

        return self.target

    def spoolUp(self):
        """
        This is the function that saturates the motion buffer such that the vehicle can be placed in offboard mode
        :return: Void method. Publishes to the necessary ROS nodes.
        """
        for i in range(20):
            self.move(self.currentPose.pose.position.x, self.currentPose.pose.position.y, self.currentPose.pose.position.z, True)

    def move(self, x, y, z, BODY_OFFSET_ENU=True):
        """
        This is the method that interfaces with the set_pose method to publish the desired motion from the UAS. As of
        now, it is functionally identical to the set_pose method, but if any edge case handling needs to be implemented,
        this is the method that would do it.
        :param x: The shift in the forward direction of the vehicle (forward is positive, FLU frame definition)
        :param y: The shift in the sideways direction of the vehicle (left is positive, FLU frame definition)
        :param z: The shift in the vertical axis of the vehicle (up is positive, FLU frame definition)
        :param BODY_OFFSET_ENU: The frame definition of the vehicle. Either FLU or ENU. Currently only FLU works. Sorry.
        :return: Void method. It publishes to the necessary ROS node.
        self.rate.sleep()
        """
        self.position_target_pub.publish(self.set_pose(x, y, z, BODY_OFFSET_ENU))

    def turn(self, yaw_degree):
        """
        This is the method that would induce any yaw update to be sent to the flight computer.
        :param yaw_degree: Yaw change in degrees
        :return: Void method. It publishes to the necessary ROS node.
        """
        self.yaw_target_pub.publish(yaw_degree)

    def land(self):
        """
        This is the method that would send a landing command to the UAS
        :return: Void method. It publishes to the necessary ROS node.
        """
        self.custom_activity_pub.publish(String("LAND"))

    def hover(self):
        """
        This is the method that would send the vehicle into a hover flight state. This causes the vehicle to fall out of
        offboard mode, and currently there is no method that would automatically put the vehicle back in offboard mode
        :return: Void method. It publishes to the necessary ROS node.
        """
        self.custom_activity_pub.publish(String("HOVER"))

    def return_home(self, height):
        """
        This is the method that would effectively reset the flight to the location where the EKF was reset. Use only in
        areas where there is sufficient room for error as this can be an error prone maneuver
        :param height: This is the height that the vehicle will be hovering at in meters
        :return: Void method. It publishes to the necessary ROS node.
        """
        self.position_target_pub.publish(self.set_pose(0, 0, height, False))

    def update_pose(self, msg):
        """
        This is the method that internally update the Pose reference for any calculation that needs to be done
        :param msg: The incoming PoseStamped message from the subscriber that is updating the internal reference.
        :return: Void method. It updates the internal currentPose variable.
        """
        self.currentPose = msg

    def svgs_motion(self, msg):
        """
        This is the callback to command any motion calculated from the SVGS nodes
        :param msg: The incoming Point message from SVGS nodes.
        :return: Void method. It calls the 'move' method with the shift corresponding to the Point from the SVGS node
        """
        self.move(msg.x, msg.y, msg.z)


if __name__ == "__main__":
    try:
        con = Commander()
        while not rospy.is_shutdown():
            con.land()
            con.rate.sleep()
    except rospy.ROSInterruptException:
        pass
