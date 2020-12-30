#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from survey_and_rescue.msg import *
from cv_bridge import CvBridge, CvBridgeError
import random
import pickle
#import imutils
import copy

class sr_determine_colors():

	def __init__(self):
		self.detect_info_msg = SRInfo()
		self.bridge = CvBridge()
		self.detect_pub = rospy.Publisher("/detection_info",SRInfo,queue_size=1) 
 		self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color",Image,self.image_callback)
 		self.serviced_sub = rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)
 		self.img=None
 		self.rect_list=None
 		self.img_copy = None
 		self.olddata=[]
 		self.dict={}
 		

	def load_rois(self, file_path = 'rect_info.pkl'):
		try:
			with open('/home/nitish/catkin_ws/src/survey_and_rescue/scripts/contourPickle.pickle', 'rb') as input:
				self.rect_list = pickle.load(input)
				print("no of cont",len(self.rect_list))
		except IOError, ValueError:
			print("File doesn't exist or is corrupted")


 	def image_callback(self, data):
 		try:
 			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
 		except CvBridgeError as e:
 			print(e)


 	def serviced_callback(self, msg):
 		pass
 	
	def detect_color_contour_centers(self,boolean):

		self.img_copy = self.img[::]
		grey = cv.cvtColor(self.img_copy,cv.COLOR_BGR2GRAY)    
		_,th=cv.threshold(grey,220,255,cv.THRESH_BINARY)
		data=[]
		_,contours,_=cv.findContours(th, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
		centers=[]
		for c in contours:
			if cv.contourArea(c) > 50:
				m=cv.moments(c)
				cx=int(m['m10']/m['m00'])
				cy=int(m['m01']/m['m00'])
				centers.append((cx,cy))
		for x,y in centers:
			for cont,pos in self.rect_list:
				check = cv.pointPolygonTest(cont, (x,y), False)
				bound = cv.boundingRect(cont)
				post=[]
				if int(check)==1:
					crop = self.img_copy[int(bound[1]):int(bound[1]+bound[3]),int(bound[0]):int(bound[0]+bound[2])]
					blue,green,red = cv.split(self.img_copy[int(bound[1]):int(bound[1]+bound[3]),int(bound[0]):int(bound[0]+bound[2])])
					b = np.sum(blue)/len(blue)
					g = np.sum(green)/len(green)
					r = np.sum(red)/len(red)
					#cv.rectangle(self.img_copy, (int(bound[0]), int(bound[1])), \
						#(int(bound[0]+bound[2]), int(bound[1]+bound[3])), (255,255,255), 3)
					if(max(r,b,g)==g):
						data.append(("FOOD",pos))
					if(max(r,b,g)==b):
						data.append(("MEDICINE",pos))
					if(max(r,b,g)==r):
						data.append(("RESCUE",pos))
		data=list(set(data))
		data_copy = data[:]
		for i in data:
			if not (i in self.dict.keys()):
				self.dict[i]=rospy.get_time()
			else:
				if i[0]=="RESCUE":
					duration=self.dict[i]+11.0-rospy.get_time()
				else:
					duration=self.dict[i]+31.0-rospy.get_time()
				if duration > 0:
					data_copy.remove(i)
				else:
					self.dict[i]=rospy.get_time()
		#data=sorted(data,key=lambda x:x[1])
		#if self.olddata != data:
			#olddata1=set(self.olddata)
			#data1=set(data)
			#new=olddata1.intersection(data1)
			#new=list(data1-new)
			#new = sorted(new,key=lambda x:x[0],reverse=True)
			#print(new)
		rate=rospy.Rate(50)
		if len(data_copy)!=0:
			msg=SRInfo()
			for info,position in data_copy:
				msg.info=info
				msg.location=position
				self.detect_pub.publish(msg)
				rate.sleep()
			#del self.olddata[:]
			#self.olddata=data[:]
							
 
	def check_whether_lit(self):
		pass


def main(args):
	
	try:
		rospy.init_node('sr_beacon_detector', anonymous=False)
		s = sr_determine_colors()
		rate = rospy.Rate(20)
		#rate = rospy.Rate(30)
		s.load_rois()
		while s.img is None:
			pass
		boolean=True
	except KeyboardInterrupt:
		cv.destroyAllWindows()
	while not rospy.is_shutdown():
		try:
			s.detect_color_contour_centers(boolean)
			boolean=False
			s.check_whether_lit()
			rate.sleep()
		except KeyboardInterrupt:
			cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)