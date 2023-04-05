#! /usr/bin/env python

import sys
import rospy 
import time
import os
import csv
from UI_nodes_pkg.msg import *
import moveit_commander
from moveit_msgs.msg import *
import math
import random

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('fake_sensors')
forceR_pub = rospy.Publisher('sensors/force_right', forces, queue_size=1)
forceL_pub = rospy.Publisher('sensors/force_left', forces, queue_size=1)
tactileR_pub = rospy.Publisher('sensors/tactile_right', tactile, queue_size=1)
tactileL_pub = rospy.Publisher('sensors/tactile_right', tactile, queue_size=1)

rate=rospy.Rate(10) #20Hz
force_msgR = forces()
force_msgL = forces()
tactile_msg = tactile()

while not rospy.is_shutdown():
	force_msgR.Fx = max(min(force_msgR.Fx + random.randint(-19,19), 99),-99)
	force_msgR.Fy = max(min(force_msgR.Fy + random.randint(-19,19), 99),-99)
	force_msgR.Fz = max(min(force_msgR.Fz + random.randint(-19,19), 99),-99)
	force_msgR.Tx = max(min(force_msgR.Tx + random.randint(-19,19), 99),-99)
	force_msgR.Ty = max(min(force_msgR.Ty + random.randint(-19,19), 99),-99)
	force_msgR.Tz = max(min(force_msgR.Tz + random.randint(-19,19), 99),-99)
	forceR_pub.publish(force_msgR)

	force_msgL.Fx = max(min(force_msgL.Fx + random.randint(-19,19), 99),-99)
	force_msgR.Fy = max(min(force_msgL.Fy + random.randint(-19,19), 99),-99)
	force_msgL.Fz = max(min(force_msgL.Fz + random.randint(-19,19), 99),-99)
	force_msgL.Tx = max(min(force_msgL.Tx + random.randint(-19,19), 99),-99)
	force_msgL.Ty = max(min(force_msgL.Ty + random.randint(-19,19), 99),-99)
	force_msgL.Tz = max(min(force_msgL.Tz + random.randint(-19,19), 99),-99)
	forceL_pub.publish(force_msgL)

	tactile_msg.x = []
	tactile_msg.y = []
	a=(random.randint(-10,10))/10
	b=(random.randint(-20,20))/10
	c=(random.randint(0,50))/10
	for i in range(50):
		tactile_msg.x.append(i/10)
		tactile_msg.y.append(a*tactile_msg.x[-1]**2 + b*tactile_msg.x[-1] + c)
	tactileR_pub.publish(tactile_msg)

	tactile_msg.x = []
	tactile_msg.y = []
	a=(random.randint(-10,10))/10
	b=(random.randint(-20,20))/10
	c=(random.randint(0,50))/10
	for i in range(50):
		tactile_msg.x.append(i/10)
		tactile_msg.y.append(a*tactile_msg.x[-1]**2 + b*tactile_msg.x[-1] + c)
	tactileL_pub.publish(tactile_msg)

	rate.sleep()