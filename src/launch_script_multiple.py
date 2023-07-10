#! /usr/bin/env python

import sys
import copy
import rospy 
import time
import roslaunch
from std_srvs.srv import Trigger, TriggerResponse
from UI_nodes_pkg.srv import *
from UI_nodes_pkg.msg import *

rospy.init_node('launch_node', anonymous=True)
rate = rospy.Rate(10)

"""
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/remodel/catkin_ws/src/pruebas_pkg/launch/printer.launch"])
launch.start()
rospy.loginfo("started")

rospy.sleep(5)
# 5 seconds later
launch.shutdown()
"""

launch_file_service = 'UI_launcher/launch_multiple'
close_file_service = 'UI_launcher/stop_multiple'
to_launch = []
to_stop = []
launch_ = {}

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
#launch_['pruebas_pkg/launch_script'] = roslaunch.parent.ROSLaunchParent(uuid, ["/home/remodel/catkin_ws/src/pruebas_pkg/launch/launch_script.launch"], sigint_timeout=15.0, sigterm_timeout=2.0)

def launch_file_service_callback(req): 
    """
    Launches a launch file
    """
    global to_launch
    resp = launch_multipleResponse()
    for launch_i in req.data:
        dict_name_i = launch_i.pkg + "/" + launch_i.file
        to_launch.append(dict_name_i)
    resp.success = True
    return resp

rospy.Service(launch_file_service, launch_multiple, launch_file_service_callback)


def close_file_service_callback(req): 
    """
    Closes a launch file that is in execution
    """
    global to_stop
    resp = launch_multipleResponse()
    resp = launch_multipleResponse()
    for stop_i in req.data:
        dict_name_i = stop_i.pkg + "/" + stop_i.file
        to_stop.append(dict_name_i)
    resp.success = True
    return resp

rospy.Service(close_file_service, launch_multiple, close_file_service_callback)


while not rospy.is_shutdown(): 
    rate.sleep() 

    to_launch_copy = []
    to_launch_copy = copy.deepcopy(to_launch)

    for to_launch_copy_i in to_launch_copy:
        print("Launching file")
        if to_launch_copy_i in launch_:
            print(str(to_launch_copy_i) + " has already been started")
        else:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch_name = to_launch_copy_i.split("/")[-1]
            package_name = to_launch_copy_i[:-len(launch_name)-1]
            file_name = "/home/remodel/" + package_name + "/launch/" + launch_name + ".launch"
            launch_[to_launch_copy_i] = roslaunch.parent.ROSLaunchParent(uuid, [file_name], sigint_timeout=1.5, sigterm_timeout=0.5)
            launch_[to_launch_copy_i].start()
            to_launch.remove(to_launch_copy_i)
            rospy.loginfo(str(to_launch_copy_i) + "started")

    to_stop_copy = []
    to_stop_copy = copy.deepcopy(to_stop)
    for to_stop_copy_i in to_stop_copy:
        print("Stopping launch file " + str(to_stop_copy_i))
        if to_stop_copy_i in launch_:
            launch_[to_stop_copy_i].shutdown()
            launch_.pop(to_stop_copy_i)
        else:
            print("That file has not been started")
        to_stop.remove(to_stop_copy_i)
	
