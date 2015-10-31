#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS coutesy of code written by Adrian Rosebrock """
#import important things
import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import rospkg
import numpy as np
from geometry_msgs.msg import Twist, Vector3

class MotionDetection(object):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self):
        """ Initialize the ball tracker """
        # rospy.init_node('MotionDetection')
        self.cv_image = None 
        self.last_image=None                       # the latest image from the camera
        self.bridge = CvBridge()     
        rospack = rospkg.RosPack()
        self.shouldstop=False
        self.framecount=0

        # self.detector = cv2.FeatureDetector_create('SIFT')
        # self.extractor = cv2.DescriptorExtractor_create('SIFT')

        
    def process_image(self, img):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        #cature image every 2 seconds
        self.shouldstop = False
        self.framecount+=1
        if self.framecount%3==0:
            self.cv_image = img
            gray=cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2GRAY)
            gray=cv2.GaussianBlur(gray,(21,21),0)
            if self.last_image is None:
                self.last_image=gray

            frameChanges=cv2.absdiff(self.last_image,gray)
            thresh=cv2.threshold(frameChanges,25,255,cv2.THRESH_BINARY)[1]
            thresh=cv2.dilate(thresh,None,iterations=2)
            (cnts, _)=cv2.findContours(thresh.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

            for c in cnts:
                print cv2.contourArea(c)
                if cv2.contourArea(c)>500:
                    self.shouldstop=True
            
            if self.shouldstop ==False:
                return self.shouldstop

                (x,y,w,h)=cv2.boundingRect(c)
                cv2.rectangle(self.cv_image,(x,y),(x+w,y+h),(0,255,0),2)

                    #print "should stop"
            

                #if shouldstop=True, then stop for three seconds, else keep going straight.

  #the code below were used to visualize and test the code, but we don't need to use this.
                      #if I don't see an object, I want to keep moving straight, don't need this once the pieces are together
                
            cv2.imshow('video_window', self.cv_image)
            self.last_image=gray
         
            cv2.waitKey(5)
        return self.shouldstop

     

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
        	# start out not issuing any motor commands
            r.sleep()

if __name__ == '__main__':
    node = MotionDetection("/camera/image_raw")
    node.run()