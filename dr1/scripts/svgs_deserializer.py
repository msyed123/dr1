#!/usr/bin/env python

import serial
import struct
import math
import time
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

sequentialFails = 0

print("Started\n")


# noinspection PyBroadException
class Slip:
    byteMsg = bytearray()
    END = b'\xc0'  # 192
    ESC = b'\xdb'  # 219
    ESC_END = b'\xdc'  # 220
    ESC_ESC = b'\xdd'  # 221

    started = False
    escaped = False

    vectors = []

    def toVector(self):
        try:
            for i in range(7):
                self.vectors += struct.unpack('f', self.byteMsg[1 + (i * 4):1 + ((i + 1) * 4)])  # byteMsg[0] is svgs mode
        except:
            Exception("PACKET FAILED")

    def zeroCheck(self):
        if SLIP.vectors[0] == 0.0 and SLIP.vectors[1] == 0.0 and SLIP.vectors[2] == 0.0:
            return False
        else:
            self.vectors.append(time.time())
            return True

    def readSerial(self, port):
        new_byte = bytearray()
        try:
            new_byte = port.read()
        except:
            print("could not read")
            raise Exception("COULD NOT READ")
        # END
        if new_byte == self.END:
            if self.started:
                return True
            else:
                self.started = True

        # ESC
        elif new_byte == self.ESC:
            self.escaped = True

            # ESC_END
        elif new_byte == self.ESC_END:
            if self.escaped:
                self.byteMsg += self.END
                self.escaped = False
            else:
                self.byteMsg += new_byte

        # ESC_ESC
        elif new_byte == self.ESC_ESC:
            if self.escaped:
                self.byteMsg += self.ESC
                self.escaped = False
            else:
                self.byteMsg += new_byte

        # ALL OTHERS
        else:
            if self.escaped:
                self.byteMsg = ''
                self.escaped = False
                raise Exception('Slip Protocol Error')
            else:
                if self.started:
                    self.byteMsg += new_byte
                else:
                    Exception('COMM ERROR')


########################################################################################################################


# original baud rate = 57600
port = serial.Serial("/dev/ttyAMA0", baudrate=38400, timeout=1.0)

rospy.init_node('svgs_deserializer', anonymous=True)
svgs_pub = rospy.Publisher('dr1/target', PoseStamped, queue_size=1)
targetAcquisitionPub = rospy.Publisher('dr1/targetAcquired', Bool, queue_size=1)
landingCommanderPub = rospy.Publisher('dr1/landing_commander_trigger', Bool, queue_size=1)
svgs_data = PoseStamped()

msgValid = False
SLIP = Slip()

while True:
    msgValid = SLIP.readSerial(port)

    if msgValid is True and len(SLIP.byteMsg) > 1:
        # print("valid message received")
        SLIP.toVector()
        # print(SLIP.vectors)
        msgValid = False
        if SLIP.zeroCheck():
            sequentialFails = 0
            # IF(DATA_FILTER()):
            # post to ROS here
            # print(SLIP.vectors)

            # There is transposition and flipping happening here in order to get the SVGS frame to match the drone frame
            svgs_data.pose.position.x = -1.0 * SLIP.vectors[1]
            svgs_data.pose.position.y = SLIP.vectors[0]
            svgs_data.pose.position.z = -1.0 * SLIP.vectors[2]

            print("X:       %1.5f" % (-1.0 * SLIP.vectors[1]))
            print("Y:       %1.5f" % (SLIP.vectors[0]))
            print("Z:       %1.5f" % (-1.0 * SLIP.vectors[2]))
            print("P:       %1.5f" % (SLIP.vectors[3] * 180.0 / 3.14159))
            print("Q:       %1.5f" % (SLIP.vectors[4] * 180.0 / 3.14159))
            print("R:       %1.5f\n" % (SLIP.vectors[5] * 180.0 / 3.14159))
            svgs_pub.publish(svgs_data)
            targetAcquisitionPub.publish(True)
            landingCommanderPub.publish(True)
        else:
            sequentialFails += 1
            # State calculation failed
            print("SCF")
            if sequentialFails >= 3:
                targetAcquisitionPub.publish(False)
                landingCommanderPub.publish(False)

        # CLEAN UP
        SLIP.byteMsg = b''
        SLIP.vectors = []
