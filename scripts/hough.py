#!/usr/bin/env python

"""A program that finds the Hough lines of a given image"""

import cv2
import pickle
import numpy as np
import rospkg
import rospy
import math

class HoughLineDetector(object):
	"""docstring for Hough_Line_detector"""
	def __init__(self, img):
		super(HoughLineDetector, self).__init__()
		self.img = img

	def find_lines(self):
		#looks for the white parts of the image
		self.binary_image = cv2.inRange(self.img, (210,210,210), (255,255,255))
		#finds the edges of the white parts using Canny edge decection
		self.edges= cv2.Canny(self.binary_image,100,400)
		#using the houghline detector for binary images, finds the lines in the image
		lines = cv2.HoughLinesP(self.edges,.01,np.pi/180,95,minLineLength = 1, maxLineGap = 1)

		angles=[]
		#checks if lines were found
		if not lines==None:
			for i in range(len(lines[0])):
				#calculates the angle of each line
				angles.append(math.atan2(lines[0][i][0]-lines[0][i][2],lines[0][i][1]-lines[0][i][3]))
			rects=[]

			#searches through each line and sees if they are parallel and start and end in the same 
			#vertical locations. If they do, then those two ,lines define a rectangle
			for i in range(len(angles)):
				for j in range(len(angles)):
					if abs(angles[i]-angles[j])<.1 and i not == j:
						if abs(lines[0][i][0]-lines[0][j][0])<20 and abs(lines[0][i][2]-lines[0][j][2])<20:
							rects.append(((lines[0][i][0],lines[0][i][1]), (lines[0][j][2],lines[0][j][3])))
							cv2.rectangle(self.img, (lines[0][i][0],lines[0][i][1]), (lines[0][j][2],lines[0][j][3]), (255,0,0), 10, 8, 0) 
			#checks to see if the rectangles are on the ground
			for rect in rects:
				for i in range(2):
					if rect[i][1]>350:
						#sees rectangles
						return True
		#doesn't see rectangles
		return False





		
if __name__ == '__main__':
	rospack = rospkg.RosPack()

	img_file = rospack.get_path('cv_proj') + '/scripts/' + "green.jpg"
	img=cv2.imread(img_file)

	node = HoughLineDetector(img)

	
	node.find_lines()
