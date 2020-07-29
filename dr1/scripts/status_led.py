#!/usr/bin/env python

import rospy
from gpiozero import LED
from std_msgs.msg import Bool

led = LED(26)
launchTime = rospy.get_param("/launch_time")
launchTime += 5

rospy.init_node("landing_commander")
time.sleep(launchTime)


def ledUpdate(msg):
    if msg.data is True:
        led.on()
    else:
        led.off()


triggerSub = rospy.Subscriber('dr1/landing_commander_trigger', Bool, ledUpdate)

if __name__ == "__main__":
    rospy.spin()
