#! /usr/bin/env python
import sys
import rospy
import actionlib
from std_msgs.msg import *
from UI_nodes_pkg.msg import process_UIAction, process_UIGoal

saved_index = 0
saved_step1 = 0
saved_step2 = 0
saved_auto = True

rospy.init_node('process_action_client')
index_publisher = rospy.Publisher('/UI/process_index', Int32, queue_size=1)
logs_publisher = rospy.Publisher('/UI/logs', String, queue_size=1)
rate=rospy.Rate(1) #1Hz
client=actionlib.SimpleActionClient('/ElvezProcess_as', process_UIAction) #Stablishes the connection with the server
client.wait_for_server() #Waits until the action server is available

def start_callback(data):
	global client
	global saved_index
	global saved_step1
	global saved_step2
	global saved_auto
	saved_index = 0
	saved_step1 = 0
	saved_step2 = 0
	goal=process_UIGoal()
	goal.index=0
	goal.subindex=0
	goal.subindex2=0
	goal.auto = True
	saved_auto = True
	msg_log = String()
	msg_log.data = "Process started in automatic mode"
	logs_publisher.publish(msg_log)
	client.send_goal(goal, feedback_cb=feedback_callback)

subsL = rospy.Subscriber('/UI/start', Bool, start_callback)  

def resume_callback(data):
	global client
	global saved_index
	global saved_step1
	global saved_step2
	global saved_auto
	goal=process_UIGoal()
	goal.index=saved_index
	goal.subindex=saved_step1
	goal.subindex2=saved_step2
	goal.auto=saved_auto
	msg_log = String()
	msg_log.data = "Process resumed in operation " + str(saved_index)
	logs_publisher.publish(msg_log)
	client.send_goal(goal, feedback_cb=feedback_callback)

subsL = rospy.Subscriber('/UI/resume', Bool, resume_callback) 

def stop_callback(data):
	global saved_index
	global client
	# goal=process_UIGoal()
	# goal.index=0
	msg_log = String()
	msg_log.data = "Process stopped"
	logs_publisher.publish(msg_log)
	client.cancel_goal()
	saved_index = 0

subsL = rospy.Subscriber('/UI/stop', Bool, stop_callback)

def pause_callback(data):
	global client
	# goal=process_UIGoal()
	# goal.index=0
	msg_log = String()
	msg_log.data = "Process paused"
	logs_publisher.publish(msg_log)
	client.cancel_goal()

subsL = rospy.Subscriber('/UI/pause', Bool, pause_callback)

def step_callback(data):
	global client
	global saved_index
	global saved_step1
	global saved_step2
	global saved_auto
	goal=process_UIGoal()
	saved_index = data.data
	saved_step1 = 0
	saved_step2 = 0
	goal.index=saved_index
	goal.subindex=0
	goal.subindex2=0
	goal.auto=False
	saved_auto = False
	msg_log = String()
	msg_log.data = "Started execution of process operation " + str(saved_index)
	logs_publisher.publish(msg_log)
	client.send_goal(goal, feedback_cb=feedback_callback)


subsL = rospy.Subscriber('/UI/step', Int32, step_callback)

def feedback_callback(feedback):
	#This function is called when the server sends feedback
	global saved_index
	global saved_step1
	global saved_step2
	saved_index = feedback.index
	saved_step1 = feedback.subindex
	saved_step2 = feedback.subindex2
	print(saved_index)
	msg = Int32()
	msg.data = saved_index
	index_publisher.publish(msg)

rospy.spin()