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
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import json

def talker():

	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(1)

	with open("/home/nitish/catkin_ws/src/survey_and_rescue/scripts/cell_coords.json", "r") as read_file:
			cords = json.load(read_file)
	command_pub = rospy.Publisher('/whycon/poses', PoseArray, queue_size=1)
	msg = PoseArray()
	ans = raw_input('enter location: ')
	print(ans)
	p=Pose()
	p.position.x=cords[ans][0]
	p.position.y=cords[ans][1]
	p.position.z=cords[ans][2]
	msg.poses.append(p)
	#msg.poses[0].position.y = cords[ans][1]
	#msg.poses[0].position.z = cords[ans][2]
	msg.header.frame_id = 'whycon'
	while not rospy.is_shutdown():
		command_pub.publish(msg)
		rate.sleep()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass	