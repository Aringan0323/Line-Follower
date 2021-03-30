#!/usr/bin/env python

import rospy
import time
import sys
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class LineFollower:

    def __init__(self):

        self.move = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.twist = Twist()

        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.mask_callback)

        #CvBridge to convert rospy topic image into a numpy array
        self.bridge = CvBridge()

        #Boolean indicating whether or not the line has been initially located
        self.line_located = False

        #Boolean indicating whether or not the line is immediately in front of the robot
        self.in_view = True

        self.mask = None


    '''
    Defines a cv2 mask that only contains pixels where the input
    image is yellow.
    '''
    def mask_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([ 20,100,100])
            upper_yellow = np.array([ 30,255,255])
            self.mask = cv2.inRange(hsv,  lower_yellow, upper_yellow)
            self.h, self.w, _ = cv_img.shape
        except CvBridgeError as e:
            print(e)

    '''
    Computes the centroid of the section of the mask just at the bottom of the mask
    and just at the top of the mask. Then defines the error between the middle of the 
    screen offset by the amount that the camera is offset from the front of the robot,
    and the centroid of the yellow line defined in the bottom of the mask.
    '''
    def compute_centroid(self):
            # Compute the "centroid" and display a red circle to denote it

        search_top = 3 * self.h /4
        search_bot = search_top + 20
        cropped_mask = np.copy(self.mask)
        cropped_mask[0:search_top, 0:self.w] = 0
        cropped_mask[search_bot:self.h, 0:self.w] = 0
        M = cv2.moments(cropped_mask)

        if M['m00'] > 0: #If a yellow line is detected in the cropped mask then calculate the centroid-x & err
            self.line_located = True
            self.in_view = True

            cx = int(M['m10']/M['m00'])
            self.err = (cx+120) - self.w/2
                
        else:
            self.in_view = False

    '''
    Publishes a twist command to move forward and a angular vel twist 
    command defined by the error of the angle from the line calculated
    in compute_centroid()
    '''
    def follow_line(self):
        self.twist.linear.x = 0.2
        self.twist.angular.z = -(float(self.err)/(self.w/2))* 1.57
        self.move.publish(self.twist)


    '''
    Rotates the robot until any yellow line is within
    the robot's field of view. Then it calculates the 
    centroid of the full uncropped mask and calculates the 
    error and moves towards the line until it is right in front
    of it.
    '''
    def find_line(self):
        M = cv2.moments(self.mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            self.err = (cx+120) - self.w/2
            self.follow_line()
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = 1
            self.move.publish(self.twist)

    def rotate(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 1
        self.move.publish(self.twist)


    '''
    Until the robot is directly in front of the line then it will
    use the find_line() function. Once the robot has located and moved
    to the line, it will follow it until it reaches the end, at which point it will
    rotate until it is in front of the line again and continue following the line.
    '''
    def main(self):
        self.compute_centroid()
        if self.in_view:
            self.follow_line()
        elif self.line_located:
            self.rotate()
        else:
            self.find_line()

    def halt(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.move.publish(self.twist)



    
