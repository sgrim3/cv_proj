#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3
import match_keypoints as mk

class CameraImage(object):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        self.initialized=False
        rospy.init_node('Sign_Tracker')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.good_matches=[]
        rospy.Subscriber(image_topic, Image, self.process_image, queue_size=1)
        cv2.namedWindow('video_window')

        self.kp_matcher = mk.KeyPointMatcherDemo("dest.jpg","dest.jpg","SIFT")
        self.initialized=True



    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        if self.initialized:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # binary_image = cv2.inRange(self.cv_image, (self.blue_lower_bound,self.green_lower_bound,self.red_lower_bound), (self.blue_upper_bound,self.green_upper_bound,self.red_upper_bound))
            print self.cv_image.shape
            #cv2.imshow('video_window', self.cv_image)
            #cv2.waitKey(100)

            self.kp_matcher.im2=self.cv_image

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        cv2.namedWindow('UI')
        cv2.namedWindow('MYWIN')
        cv2.setMouseCallback("MYWIN",self.mouse_event,self.kp_matcher)
        cv2.createTrackbar('Corner Threshold', 'UI', 0, 100, self.set_corner_threshold)
        cv2.createTrackbar('Ratio Threshold', 'UI', 100, 100, self.set_ratio_threshold)
        while not rospy.is_shutdown():
               
            self.good_matches=self.kp_matcher.compute_matches()
            if len(self.good_matches)>8:
                print "STOOOOOOOOOOOOOOP"
            else:
                print "GOOOOOOOOOOOOOOOOO"
            # cv2.resize(self.kp_matcher.im,(50,100))
            cv2.imshow("MYWIN",self.kp_matcher.im)

            # while True:
            #     cv2.imshow("MYWIN",self.kp_matcher.im)
            cv2.waitKey(50)
            r.sleep()
        cv2.destroyAllWindows()
        
    def set_corner_threshold(self,thresh):
        """ Sets the threshold to consider an interest point a corner.  The higher the value
            the more the point must look like a corner to be considered """
        self.kp_matcher.corner_threshold = thresh/1000.0

    def set_ratio_threshold(self,ratio):
        """ Sets the ratio of the nearest to the second nearest neighbor to consider the match a good one """
        
        self.kp_matcher.ratio_threshold = ratio/100.0

    def mouse_event(self,event,x,y,flag,im):
        """ Handles mouse events.  In this case when the user clicks, the matches are recomputed """
        if event == cv2.EVENT_FLAG_LBUTTON:
            self.good_matches = self.kp_matcher.compute_matches()
            print self.good_matches
if __name__ == '__main__':
    node = CameraImage("/camera/image_raw")
    node.run()

