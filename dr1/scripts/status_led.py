#!/usr/bin/env python

import rospy
from gpiozero import LED
from std_msgs.msg import Bool
import time

led = LED(26)
launchTime = rospy.get_param("/launch_time")
launchTime += 5

time.sleep(launchTime)
rospy.init_node("status_led")

# Flash the LEDs to let the user know when the system is ready to boot
for i in range(3):
    led.on()
    time.sleep(0.33)
    led.off()
    time.sleep(0.33)


def ledUpdate(msg):
    if msg.data is True:
        led.on()
    else:
        led.off()


triggerSub = rospy.Subscriber('dr1/targetAcquired', Bool, ledUpdate)

if __name__ == "__main__":
    rospy.spin()

