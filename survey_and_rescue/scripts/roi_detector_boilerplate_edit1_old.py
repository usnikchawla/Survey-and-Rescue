#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pickle
import imutils
import copy
import numpy as np
import itertools

class sr_determine_rois():

	def __init__(self):

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/usb_cam/image_rect",Image,self.image_callback)
		self.img = None
	
	# CV_Bridge acts as the middle layer to convert images streamed on rostopics to a format that is compatible with OpenCV
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "mono8")
		except CvBridgeError as e:
			print(e)


	'''You will need to implement an image processing algorithm to detect the Regions of Interest (RoIs)
	The standard process flow is:
	i)		Standard pre-processing on the input image (de-noising, smoothing etc.)
	ii)		Contour Detection
	iii)	Finding contours that are square, polygons with 4 vertices
	iv)		Implementing a threshold on their area so that only contours that are the size of the cells remain'''
	def detect_rois(self):
		#im = cv2.imread('test.jpg') #contour detect
        self.img_copy = self.img
        src_gray = cv.cvtColor(self.img_copy, cv.COLOR_BGR2GRAY)
        src_gray = cv.blur(src_gray, (3,3))
        source_window = 'Source'
        cv.namedWindow(source_window)
        cv.imshow(source_window, self.img_copy)
        thresh = 100 # initial threshold
        thresh_callback(thresh)
        canny_output = cv.Canny(src_gray, threshold, threshold * 2)
        
        _, contours, _ = cv.findContours(canny_output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
        contours_poly = [None]*len(contours)
        boundRect = [None]*len(contours)
    
        for i, c in enumerate(contours):
            contours_poly[i] = cv.approxPolyDP(c, 3, True)
            boundRect[i] = cv.boundingRect(contours_poly[i])
            
        drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)
    
        for i in range(len(contours)):
            color=(256,256,256)
            cv.drawContours(drawing, contours_poly, i, color)
            cv.rectangle(drawing, (int(boundRect[i][0]), int(boundRect[i][1])), \
                (int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])), color, 3) #why both?
        
        cv.imshow('Contours', drawing)

 		#imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
		#ret, thresh = cv2.threshold(imgray, 127, 255, 0)
		#im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                
		# Add your Code here
		# You may add additional function parameters
		# cv2.imshow("Detected ROIs", img_copy) #A copy is chosen because self.img will be continuously changing due to the callback function
		# cv2.waitKey(100)


	'''Please understand the order in which OpenCV detects the contours.Before saving the files you may sort them, so that their order corresponds will the cell order This will help you greatly in the next part.'''
	def sort_rois(self):
		# Add your Code here

		

	def query_yes_no(self, question, default=None):
		"""Ask a yes/no question via raw_input() and return their answer.

		"question" is a string that is presented to the user.
		"default" is the presumed answer if the user just hits <Enter>.
		It must be "yes" (the default), "no" or None (meaning
		an answer is required of the user).

		The "answer" return value is True for "yes" or False for "no".
		"""
		valid = {"yes": True, "y": True, "ye": True,"no": False, "n": False}
		if default is None:
			prompt = " [Y/N]:\t"
		elif default == "yes":
			prompt = " [Y/N]:\t"
		elif default == "no":
			prompt = " [Y/N]:\t"
		else:
			raise ValueError("Invalid default answer: '%s'" % default)

		while True:
			sys.stdout.write(question + prompt)
			choice = raw_input().lower()
			if default is not None and choice == '':
				return valid[default]
			elif choice in valid:
				return valid[choice]
			else:
				sys.stdout.write("\nPlease respond with 'yes' or 'no' ""(or 'y' or 'n').\n")

	'''You may save the list using anymethod you desire
	   The most starightforward way of doing so is staright away pickling the objects
		You could also save a dictionary as a json file, or, if you are using numpy you could use the np.save functionality
		Refer to the internet to find out more '''
	def save_rois(self):
		#Add your code here

	#You may optionally implement this to display the image as it is displayed in the Figure given in the Problem Statement
	def draw_cell_names(self, img):
		#Add your code here

def main(args):
	#Sample process flow
	try:
		rospy.init_node('sr_roi_detector', anonymous=False)
		r =	sr_determine_rois()
		while True:
			if r.img is not None:
				r.detect_rois()
				if('''No of cells detected is not 36'''):
					new_thresh_flag = r.query_yes_no("36 cells were not detected, do you want to change ##Enter tweaks, this is not necessary##?")
					if(new_thresh_flag):
						#Change settings as per your desire
					else:
						continue
				else:
					satis_flag = r.query_yes_no("Are you satisfied with the currently detected ROIs?")
					if(satis_flag):
						r.sort_rois()
						r.save_rois()
						cv2.destroyAllWindows()
						break
					else:
						#Change more settings
		# r.draw_cell_names(r.img) # Again, this is optional
	except KeyboardInterrupt:
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
