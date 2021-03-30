#!/bin/bash
rosbag record /mavros/local_position/velocity_body /camera/odom/sample_throttled /dr1/current_error /dr1/current_velocity /dr1/kfPosition /dr1/kfVelocity /dr1/landing_counter /dr1/landing_flag /dr1/target /dr1/targetAcquired /dr1/velocity_setpoint /mavros/state
