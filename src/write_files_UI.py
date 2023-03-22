#! /usr/bin/env python

import sys
import rospy 
import time
import os
import csv
from std_srvs.srv import *
from UI_nodes_pkg.srv import *
from UI_nodes_pkg.msg import *

rospy.init_node('config_node', anonymous=True)
write_config_file_service = '/UI/write_config_file'
get_config_service = '/UI/get_config'
set_config_service = '/UI/set_config'
path_current = os.path.dirname(__file__)
path_config = os.path.join(path_current, '../files/config.csv')



config_dict = {}
fieldnames = ['prop', 'value']

with open(path_config) as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        config_dict[row['prop']]=row['value']


def get_config_callback(req): 
	resp=GetConfigResponse()
	resp.success = True
	try:
		path_config_srv = os.path.join(path_current, req.file_name)
		with open(path_config_srv) as csvfile:
			reader = csv.DictReader(csvfile)
			for row in reader:
				resp_el = configProp()
				resp_el.prop = row['prop']
				resp_el.value = row['value']
				resp.data.append(resp_el)
	except:
		resp.success = False
	return resp

rospy.Service(get_config_service, GetConfig, get_config_callback) #Change srv type


def set_config_callback(req): 
	resp=SetConfigResponse()
	resp.success = True
	try:
		for el in req.data:
			config_dict[el.prop] = el.value

		rows=[]
		for prop in config_dict:
			rows.append({'prop': prop, 'value': config_dict[prop]})

		path_config_srv = os.path.join(path_current, req.file_name)
		with open(path_config_srv, 'w') as f:
			writer = csv.DictWriter(f, fieldnames=fieldnames)
			writer.writeheader()
			writer.writerows(rows)
	except:
		resp.success = False
	return resp

rospy.Service(set_config_service, SetConfig, set_config_callback) #Change srv type


def write_config_file_callback(req): 
	resp=WriteConfigResponse()
	resp.success = True
	fieldnames = ['prop', 'value']
	try:
		config_dict[req.prop] = req.value

		rows=[]
		for prop in config_dict:
			rows.append({'prop': prop, 'value': config_dict[prop]})

		with open(req.file_name, 'w') as f:
			writer = csv.DictWriter(f, fieldnames=fieldnames)
			writer.writeheader()
			writer.writerows(rows)
	except:
		resp.success = False
	return resp

rospy.Service(write_config_file_service, WriteConfig, write_config_file_callback) #Change srv type

# rows = [
#     {'prop': 'speed_fast', 'value': 50},
#     {'prop': 'speed_medium', 'value': 30},
#     {'prop': 'speed_slow', 'value': 20},
#     {'prop': 'offset_x', 'value': 10},
# ]

rospy.spin()