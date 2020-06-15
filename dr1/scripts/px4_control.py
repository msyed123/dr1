#!/usr/bin/env python

#Importing the required libraries 
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64, String, Header
import time
from pyquaternion import Quaternion
import math
import threading


class Px4Controller: #Class is used for creating objects 
    def __init__(self): #Defining the function

        """
        “self” keyword access the attributes and methods of the class in python
         IMU is "Inertial Measurement Unit"
         On the right side of '=' sign, 'None' means that we dont have the values of the specific task now
        """
        self.imu = None 
        self.gps = None 
        self.local_pose = None 
        self.current_state = None
        self.current_heading = None
        self.takeoff_height = 2   # Takeoff height value 
        self.local_enu_position = None

        self.cur_target_pose = None
        self.global_target = None

        self.received_new_task = False
        self.arm_state = False
        self.offboard_state = False
        self.received_imu = False
        self.frame = "BODY"

        self.state = None

        '''
        ros subscribers
        '''
        #A node that wants to receive that information uses a subscriber to that same topic
        ##Posestamped means the value will be stored at some specific time 
        ####'NavSatFix' is for Navigation Satellite fix status for any Global Navigation Satellite System
        #####'Imu' holds data from an IMU (Inertial Measurement Unit)
        ###### 'PoseStamped' is a Pose with reference coordinate frame and timestamp
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        #Set position set point 
        self.set_target_position_sub = rospy.Subscriber("dr1/set_pose/position", PoseStamped,
                                                        self.set_target_position_callback)
        self.set_target_yaw_sub = rospy.Subscriber("dr1/set_pose/orientation", Float32, self.set_target_yaw_callback)
        self.custom_activity_sub = rospy.Subscriber("dr1/set_activity/type", String, self.custom_activity_callback)

        '''
        ros publishers 
        '''
        #If a node wants to share information, it uses a publisher to send data to a topic
        ### View position and velocity setpoint
        #### 'queue_size' is the size of the outgoing message queue used for asynchronous publishing.
        self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        '''
        ros services
        '''
        #A ROS service is a client/server system.The client sends a requests, and blocks until it receives a response.
        # We should use ROS services only for computations and quick actions.
        self.armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)

        print("PX4 Controller Initialized!") # Message that needs to be displayed after the intial process

    def start(self):   
        rospy.init_node("offboard_node") #Intializing the Node 
        time.sleep(5) #Time Delay of 5 milliseconds 
         
        #Wait for a specific time 
        rospy.wait_for_service('mavros/cmd/arming') 
        rospy.wait_for_service('mavros/set_mode')
        rospy.wait_for_service('mavros/cmd/takeoff')

        #Defining the value of "self.cur_target_pose"
        self.cur_target_pose = self.construct_target(0, 0, self.takeoff_height, self.current_heading)

        # print ("self.cur_target_pose:", self.cur_target_pose, type(self.cur_target_pose))
        
        # For, if, and else function
        for i in range(10):
            self.local_target_pub.publish(self.cur_target_pose)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            time.sleep(0.2)

        if self.takeoff_detection():
            print("Vehicle Took Off!")

        else:
            print("Vehicle Took Off Failed!")
            return

        '''
        main ROS thread
        '''
        while self.arm_state and self.offboard_state and (rospy.is_shutdown() is False):
            self.local_target_pub.publish(self.cur_target_pose)
            if (self.state is "LAND") and (self.local_pose.pose.position.z < 0.15):
                if self.disarm():
                    self.state = "DISARMED"

            time.sleep(0.1)
            
    # Defining the target function in terms of x,y and z coordinates(Target)
    def construct_target(self, x, y, z, yaw, yaw_rate=1): 
        target_raw_pose = PositionTarget()
        target_raw_pose.header = Header()
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.coordinate_frame = 9

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z
         
        #Position, Velocity, Acceleration/Florce Vectors ignore flags 
        target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        return target_raw_pose

    '''
    cur_p : poseStamped
    target_p: positionTarget
    '''

    def position_distance(self, cur_p, target_p, threshold=0.05):
        delta_x = math.fabs(cur_p.pose.position.x - target_p.position.x)
        delta_y = math.fabs(cur_p.pose.position.y - target_p.position.y)
        delta_z = math.fabs(cur_p.pose.position.z - target_p.position.z)

        if (math.sqrt((delta_x ** 2) + (delta_y ** 2) + (delta_z ** 2))) < threshold:
            return True
        else:
            return False
"""
Defining functions from the ROS Subscribers for the message 
"""
    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode

    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg

        self.current_heading = self.q2yaw(self.imu.orientation)

        self.received_imu = True

    def gps_callback(self, msg):
        self.gps = msg

    def FLU2ENU(self, msg):

        FLU_x = msg.pose.position.x * math.cos(self.current_heading) - msg.pose.position.y * math.sin(
            self.current_heading)
        FLU_y = msg.pose.position.x * math.sin(self.current_heading) + msg.pose.position.y * math.cos(
            self.current_heading)
        FLU_z = msg.pose.position.z

        return FLU_x, FLU_y, FLU_z
#Defing target in terms of FLU(Forward, Left and Up) and ENU (East, North and Up)
    def set_target_position_callback(self, msg):
        print("Received New Position Task!")

        if msg.header.frame_id == 'base_link':
            '''
            BODY_FLU
            '''
            # For Body frame, we will use FLU (Forward, Left and Up)
            #           +Z     +X
            #            ^    ^
            #            |  /
            #            |/
            #  +Y <------body

            self.frame = "BODY"

            print("body FLU frame")

            ENU_X, ENU_Y, ENU_Z = self.FLU2ENU(msg)

            ENU_X = ENU_X + self.local_pose.pose.position.x
            ENU_Y = ENU_Y + self.local_pose.pose.position.y
            ENU_Z = ENU_Z + self.local_pose.pose.position.z

            self.cur_target_pose = self.construct_target(ENU_X,
                                                         ENU_Y,
                                                         ENU_Z,
                                                         self.current_heading)

        else:
            '''
            LOCAL_ENU
            '''
            # For world frame, we will use ENU (EAST, NORTH and UP)
            #     +Z     +Y
            #      ^    ^
            #      |  /
            #      |/
            #    world------> +X

            self.frame = "LOCAL_ENU"
            print("local ENU frame")

            self.cur_target_pose = self.construct_target(msg.pose.position.x,
                                                         msg.pose.position.y,
                                                         msg.pose.position.z,
                                                         self.current_heading)

    '''
     Receive A Custom Activity
     '''
    
    def custom_activity_callback(self, msg): #Defining function for the custom activities  

        print("Received Custom Activity:", msg.data)

        if msg.data == "LAND":
            print("LANDING!")
            self.state = "LAND"
            self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                         self.local_pose.pose.position.y,
                                                         0.1,
                                                         self.current_heading)

        if msg.data == "HOVER":
            print("HOVERING!")
            self.state = "HOVER"
            self.hover()

        else:
            print("Received Custom Activity:", msg.data, "not supported yet!")
    
    def set_target_yaw_callback(self, msg): #Defining function for yaw 
        print("Received New Yaw Task!")

        yaw_deg = msg.data * math.pi / 180.0
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.local_pose.pose.position.z,
                                                     yaw_deg)

    '''
    return yaw from current IMU.
    '''

    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vehicle Offboard failed")
            return False

    def hover(self):

        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.local_pose.pose.position.z,
                                                     self.current_heading)

    def takeoff_detection(self):
        if self.local_pose.pose.position.z > 0.1 and self.offboard_state and self.arm_state:
            return True
        else:
            return False


if __name__ == '__main__':
    con = Px4Controller()
    con.start()
