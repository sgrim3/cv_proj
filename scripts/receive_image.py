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
import match_keypoints1 as mk
import hough
import rospkg
import MotionDetection1 as md

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
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()

        self.kp_matcherStop = mk.KeyPointMatcherDemo("dest.jpg","dest.jpg","SIFT")
        self.kp_matcherYield = mk.KeyPointMatcherDemo("yield.jpg","yield.jpg","SIFT")
        self.motionDetector = md.MotionDetection()
        self.initialized=True
        self.seesYieldSign = False
        self.seesStopSign = False
        self.see_rect_last =False
        

        rospack = rospkg.RosPack()
        img_file = rospack.get_path('cv_proj') + '/scripts/' + "green.jpg"
        img=cv2.imread(img_file)
        self.hough_finder = hough.HoughLineDetector(img)

        self.initialized=True

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        if self.initialized:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            self.kp_matcherStop.im2=self.cv_image
            self.kp_matcherYield.im2 = self.cv_image

            self.hough_finder.img=self.cv_image

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(10)
        # cv2.namedWindow('UI')
        #cv2.namedWindow('MYWIN')
        # cv2.setMouseCallback("MYWIN",self.mouse_event,self.kp_matcher)
        # cv2.createTrackbar('Corner Threshold', 'UI', 0, 100, self.set_corner_threshold)
        # cv2.createTrackbar('Ratio Threshold', 'UI', 100, 100, self.set_ratio_threshold)

        # cv2.createTrackbar('Red Lower', 'UI', 0, 255, self.set_red_lower_bound)
        # cv2.createTrackbar('Red Upper', 'UI', 255, 255, self.set_red_upper_bound)
        # cv2.createTrackbar('Blue Lower', 'UI', 0, 255, self.set_blue_lower_bound)
        # cv2.createTrackbar('Blue Upper', 'UI', 255, 255, self.set_blue_upper_bound)
        # cv2.createTrackbar('Green Lower', 'UI', 0, 100, self.set_green_lower_bound)
        # cv2.createTrackbar('Green Upper', 'UI', 255, 255, self.set_green_upper_bound)

        #cv2.namedWindow('hough')
        while not rospy.is_shutdown():
               
            self.good_matchesStop=self.kp_matcherStop.compute_matches()
            self.good_matchesYield=self.kp_matcherYield.compute_matches()
            see_rect = self.hough_finder.find_lines()
            sees_motion = self.motionDetector.process_image(self.cv_image)

                        #TODO- change 8
            if len(self.good_matchesStop)>4 and see_rect==False and self.see_rect_last ==True and len(self.good_matchesStop)>len(self.good_matchesYield):
                self.seesStopSign = True
                self.seesYieldSign = False
                print "STOOOOOOOOOOOOOOP"
                self.twist.linear.x = 0
                self.pub.publish(self.twist)
                r.sleep()

            elif len(self.good_matchesYield)>4 and len(self.good_matchesStop)<len(self.good_matchesYield):
                self.seesYieldSign = True
                self.seesStopSign = False
                self.twist.linear.x = 0
                self.pub.publish(self.twist)
                print "YIEEEEEEEEEEELDDDDD"
                print self.twist.linear.x
                rospy.sleep(1)
                while sees_motion:
                    print "motion"
                    self.twist.linear.x = 0
                    self.pub.publish(self.twist)
                    rospy.sleep(3)
                    sees_motion = self.motionDetector.process_image(self.cv_image)
                    rospy.sleep(.01)
                    sees_motion = self.motionDetector.process_image(self.cv_image)
                    rospy.sleep(.01)
                    sees_motion = self.motionDetector.process_image(self.cv_image)
                self.twist.linear.x = 0.1
                self.pub.publish(self.twist)
                rospy.sleep(3)

            else:
                self.seesYieldSign = False
                self.seesStopSign = False
                print "GOOOOOOOOOOOOOOOOO"
                self.twist.linear.x = 0.1
                self.pub.publish(self.twist)
            

            self.see_rect_last = see_rect

            #cv2.imshow("hough",self.hough_finder.img)
            #cv2.imshow("MYWIN",self.kp_matcherStop.im)
            cv2.imshow("MYWIN1",self.kp_matcherYield.im)
            cv2.waitKey(50)
            r.sleep()
        cv2.destroyAllWindows()

        
    # def set_corner_threshold(self,thresh):
    #     """ Sets the threshold to consider an interest point a corner.  The higher the value
    #         the more the point must look like a corner to be considered """
    #     self.kp_matcher.corner_threshold = thresh/1000.0

    # def set_ratio_threshold(self,ratio):
    #     """ Sets the ratio of the nearest to the second nearest neighbor to consider the match a good one """
        
    #     self.kp_matcher.ratio_threshold = ratio/100.0

    # def set_red_lower_bound(self, val):
    #     """ A callback function to handle the OpenCV slider to select the red lower bound """
    #     self.kp_matcher.red_lower_bound = val

    # def set_green_lower_bound(self, val):
    #     """ A callback function to handle the OpenCV slider to select the green lower bound """
    #     self.kp_matcher.green_lower_bound = val

    # def set_blue_lower_bound(self, val):
    #     """ A callback function to handle the OpenCV slider to select the  blue lower bound """
    #     self.kp_matcher.blue_lower_bound = val

    # def set_red_upper_bound(self, val):
    #     """ A callback function to handle the OpenCV slider to select the red upper bound """
    #     self.kp_matcher.red_upper_bound = val

    # def set_green_upper_bound(self, val):
    #     """ A callback function to handle the OpenCV slider to select the green upper bound """
    #     self.kp_matcher.green_upper_bound = val

    # def set_blue_upper_bound(self, val):
    #     """ A callback function to handle the OpenCV slider to select the blue upper bound """
    #     self.kp_matcher.blue_upper_bound = val

    # def mouse_event(self,event,x,y,flag,im):
    #     """ Handles mouse events.  In this case when the user clicks, the matches are recomputed """
    #     if event == cv2.EVENT_FLAG_LBUTTON:
    #         self.good_matches = self.kp_matcher.compute_matches()
    #         print self.good_matches
if __name__ == '__main__':
    node = CameraImage("/camera/image_raw")
    node.run()

