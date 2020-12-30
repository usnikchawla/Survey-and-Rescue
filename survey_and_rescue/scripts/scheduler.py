#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from survey_and_rescue.msg import *
from collections import deque
from edrone_client.msg import *


class sr_scheduler():

	def __init__(self):
		rospy.Subscriber('/detection_info',SRInfo,self.detection_callback)	
		rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)
		rospy.Subscriber('/stats_sr', SRDroneStats, self.stats_callback)
		self.decision_pub = rospy.Publisher('/decision_info',SRInfo,queue_size=4)
		#self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		self.decided_msg = SRInfo()
		self.d_resc = deque() 
		self.d_med = deque()
		self.d_food = deque()
		self.flag_inserv = False
		self.base = SRInfo()
		self.base.location, self.base.info = ('E4', 'BASE')
		self.food_left = 0
		self.med_left = 0
		self.rcell = None
		self.pubdata = None
		self.Base=False

	def detection_callback(self, msg):

		if msg.info == 'RESCUE':
			self.d_resc.appendleft((msg,rospy.get_time()))
			if self.Base == False:
				self.flag_inserv=False
		elif msg.info == 'FOOD':
			self.d_food.appendleft((msg,rospy.get_time()))
		elif msg.info == 'MEDICINE':
			self.d_med.appendleft((msg,rospy.get_time()))

	def serviced_callback(self,msg):
		# Take appropriate action when either service SUCCESS or FAILIURE is recieved from monitor.pyc
		if self.pubdata == msg.location:
			if msg.location == self.rcell and msg.info == "SUCCESS":
				self.Base==True
				self.decision_pub.publish(self.base)
				self.pubdata='E4' 
				self.flag_inserv = True
			else:
				self.Base=False
				self.flag_inserv = False

	def shutdown_hook(self):
		pass
		#self.decision_pub.publish(self.base)
        #self.land.publish("land")  

	def decide_publish(self):

		food_left_copy = self.food_left
		med_left_copy = self.med_left 
		

		if self.flag_inserv == False:
			if len(self.d_resc) != 0 and (self.d_resc[len(self.d_resc) - 1][1]+10 - rospy.get_time()) >=7:
				self.rcell=self.d_resc[len(self.d_resc) - 1][0].location
				self.pubdata=self.d_resc[len(self.d_resc) - 1][0].location
				#print('publish rescue')
				self.decision_pub.publish(self.d_resc.pop()[0])
				self.flag_inserv = True

			elif (len(self.d_med) != 0) or (len(self.d_food) != 0):

				if (len(self.d_resc) != 0 and (self.d_resc[len(self.d_resc) - 1][1]+10 - rospy.get_time()) <7): 
					self.d_resc.pop()

				if len(self.d_food) != 0:
					food_tleft=self.d_food[len(self.d_food) - 1][1] + 30.0 - rospy.get_time()
					if food_tleft < 8.0:
						self.d_food.pop() 
				else:
					food_tleft=1000	
				if len(self.d_med) != 0:
					medice_tleft=self.d_med[len(self.d_med) - 1][1] + 30.0 - rospy.get_time()
					if medice_tleft < 8.0:
						self.d_med.pop() 
				else:
					medice_tleft=1000
				if (food_tleft == min(food_tleft,medice_tleft)) and (food_tleft >= 8.0) and (food_left_copy > 0):
					print(food_left_copy,end=" ")
					self.pubdata=self.d_food[len(self.d_food) - 1][0].location
					print(self.pubdata,"FOOD")
					self.decision_pub.publish(self.d_food.pop()[0])
					self.flag_inserv = True
				elif (medice_tleft >= 8.0) and (med_left_copy > 0):
					print(med_left_copy,end=" ")
					self.pubdata=self.d_med[len(self.d_med) - 1][0].location
					print(self.pubdata,"MEDICINE")
					self.decision_pub.publish(self.d_med.pop()[0])
					self.flag_inserv = True
				elif (med_left_copy == 0) or (food_left_copy==0):
					if len(self.d_resc) != 0:
						return
					rospy.sleep(.5)    
					if len(self.d_resc) != 0:
						return
					self.pubdata='E4'	
					self.Base=True	
					self.decision_pub.publish(self.base)
					self.flag_inserv = True


	def stats_callback(self, msg):
		self.food_left = msg.foodOnboard
		self.med_left = msg.medOnboard				              




def main(args):
	sched = sr_scheduler()
	rospy.init_node('sr_scheduler', anonymous=False)
	rospy.on_shutdown(sched.shutdown_hook)
	rate = rospy.Rate(30)
	while not rospy.is_shutdown():
		sched.decide_publish()
		rate.sleep()



if __name__ == '__main__':
    main(sys.argv)		