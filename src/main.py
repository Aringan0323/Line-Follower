#!/usr/bin/env python

import rospy
import time
from img_to_cmd_vel import LineFollower

if __name__ == "__main__":
    rospy.init_node('line_follower')
    follower = LineFollower()
    time.sleep(1)
    while not rospy.is_shutdown():
        rospy.sleep(0.017) #Sleep for ~1/60 seconds
        follower.main() #Loops over main control function
    follower.halt()