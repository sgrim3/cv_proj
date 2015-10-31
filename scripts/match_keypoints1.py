#!/usr/bin/env python

""" A demo that shows how keypoint matches work using SIFT """

import cv2
import pickle
import numpy as np
import rospkg


class KeyPointMatcherDemo(object):
	""" KeyPointMatcherDemo shows the basics of interest point detection,
		descriptor extraction, and descriptor matching in OpenCV """
	def __init__(self, im1_file, im2_file, descriptor_name):
		rospack = rospkg.RosPack()
		self.im1_file = rospack.get_path('cv_proj') + '/scripts/' + im1_file
		self.im2_file = rospack.get_path('cv_proj') + '/scripts/' + im2_file

		self.detector = cv2.FeatureDetector_create(descriptor_name)
		self.extractor = cv2.DescriptorExtractor_create(descriptor_name)
		self.matcher = cv2.BFMatcher()
		self.im = None


		self.corner_threshold = 20.0/1000.0
		self.ratio_threshold = 60.0/100.0
		self.red_lower_bound=20.0
		self.red_upper_bound=255.0
		self.green_lower_bound=0.0
		self.green_upper_bound=235.0
		self.blue_lower_bound=10.0
		self.blue_upper_bound=125.0

		# self.corner_threshold = 0.0
		# self.ratio_threshold = 1.0
		# self.red_lower_bound=0.0
		# self.red_upper_bound=255.0
		# self.green_lower_bound=0.0
		# self.green_upper_bound=255.0
		# self.blue_lower_bound=0.0
		# self.blue_upper_bound=255.0

		self.im1 = cv2.imread(self.im1_file)
		self.im2 = cv2.imread(self.im2_file)

	def compute_matches(self):
		""" reads in two image files and computes possible matches between them using SIFT """
		

		im1_bw = cv2.cvtColor(self.im1,cv2.COLOR_BGR2GRAY)
		im2_bw = cv2.cvtColor(self.im2,cv2.COLOR_BGR2GRAY)

		kp1 = self.detector.detect(im1_bw)
		kp2 = self.detector.detect(im2_bw)

		dc, des1 = self.extractor.compute(im1_bw,kp1)
		dc, des2 = self.extractor.compute(im2_bw,kp2)

		matches = self.matcher.knnMatch(des1,des2,k=2)


		good_matches = []
		for m,n in matches:
			# make sure the distance to the closest match is sufficiently better than the second closest
			if (m.distance < self.ratio_threshold*n.distance and
				kp1[m.queryIdx].response > self.corner_threshold and
				kp2[m.trainIdx].response > self.corner_threshold):
			
				good_matches.append((m.queryIdx, m.trainIdx))

		#for each set of pixels in the good matches, check in image if the pixel
		good_matches_color=[]

		for m,n in good_matches:
			#this gets the pixel coordinates of the point 
			#return x and y coordinates of the pixel
			pixelcoor=kp2[n].pt
			#takes in y and x
			#only append if the color range is around red color range
			#print (self.im2[pixelcoor[1],pixelcoor[0],0],self.im2[pixelcoor[1],pixelcoor[0],1])

			if (self.red_lower_bound<self.im2[pixelcoor[1],pixelcoor[0],2]<self.red_upper_bound) and (self.blue_lower_bound<self.im2[pixelcoor[1],pixelcoor[0],0]<self.blue_upper_bound) and (self.green_lower_bound<self.im2[pixelcoor[1],pixelcoor[0],1]<self.green_upper_bound):

				# print "B",self.im2[pixelcoor[1],pixelcoor[0],0]

				# print "G",self.im2[pixelcoor[1],pixelcoor[0],1]

				# print "R",self.im2[pixelcoor[1],pixelcoor[0],2]


				good_matches_color.append((m,n))


		pts1 = np.zeros((len(good_matches_color),2))
		pts2 = np.zeros((len(good_matches_color),2))
		#print good_matches

		for idx in range(len(good_matches_color)):
			match = good_matches_color[idx]
			pts1[idx,:] = kp1[match[0]].pt
			pts2[idx,:] = kp2[match[1]].pt

		self.im = np.array(np.hstack((self.im1,self.im2)))

		# plot the points
		for i in range(pts1.shape[0]):
			cv2.circle(self.im,(int(pts1[i,0]),int(pts1[i,1])),2,(255,0,0),2)
			cv2.circle(self.im,(int(pts2[i,0]+self.im1.shape[1]),int(pts2[i,1])),2,(255,0,0),2)
			cv2.line(self.im,(int(pts1[i,0]),int(pts1[i,1])),(int(pts2[i,0]+self.im1.shape[1]),int(pts2[i,1])),(0,255,0))
		return good_matches_color


