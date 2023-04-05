#! /usr/bin/env python3
import sys
import os
import copy
import rospy
import PyKDL 
import time
import moveit_commander
from moveit_msgs.msg import *
from moveit_msgs.srv import *
import geometry_msgs.msg
from std_msgs.msg import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import *
import math
from sensor_msgs.msg import *
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import xml.etree.ElementTree as ET
import tf
from elvez_pkg.msg import *
from elvez_pkg.srv import *
from force_control_routing_pkg.msg import *
import actionlib
from actionlib_msgs.msg import GoalStatusArray
from rviz_test.msg import gripper_ActFeedback, gripper_ActResult, gripper_ActAction, gripper_ActGoal
from UI_nodes_pkg.srv import *
from UI_nodes_pkg.msg import *


#ROS init
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_manual', anonymous=True)

#Define services
get_moveit_groups_service = 'UI/get_moveit_groups'
move_group_service = 'UI/move_group'
get_arms_pose_service = 'UI/get_arms_pose'

#Define boradcasters
br = tf.TransformBroadcaster() 

#Defines the movegroups. 
#IMPORTANT: These must be the name of these variables and move_groups. Otherwise, additional modifications of the code will be required.
moveit_groups = {}
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
moveit_groups["arm_left"] = moveit_commander.MoveGroupCommander("arm_left")
moveit_groups["arm_right"] = moveit_commander.MoveGroupCommander("arm_right")
moveit_groups["arms"] = moveit_commander.MoveGroupCommander("arms")
moveit_groups["torso"] = moveit_commander.MoveGroupCommander("torso")
moveit_groups["sda10f"] = moveit_commander.MoveGroupCommander("sda10f")

moveit_groups["arm_left"].clear_pose_targets()
moveit_groups["arm_right"].clear_pose_targets()
moveit_groups["arms"].clear_pose_targets()
moveit_groups["torso"].clear_pose_targets()
moveit_groups["sda10f"].clear_pose_targets()

speed_limit = 0.2
moveit_groups["arm_left"].set_max_velocity_scaling_factor(speed_limit)
moveit_groups["arm_right"].set_max_velocity_scaling_factor(speed_limit)
moveit_groups["arms"].set_max_velocity_scaling_factor(speed_limit)
moveit_groups["torso"].set_max_velocity_scaling_factor(speed_limit)
moveit_groups["sda10f"].set_max_velocity_scaling_factor(speed_limit)

#Functions
def broadcastTransform(br, frame, frame_id, parent_frame, time=rospy.get_rostime()): 
    br.sendTransform((frame.p.x(), frame.p.y(), frame.p.z()), 
        frame.M.GetQuaternion(), 
        time, 
        frame_id, 
        parent_frame) 
    
def frame_to_pose(frame):
    pose_result = Pose()
    pose_result.position.x = frame.p[0] 
    pose_result.position.y = frame.p[1] 
    pose_result.position.z = frame.p[2] 
    ang = frame.M.GetQuaternion() 
    pose_result.orientation.x = ang[0] 
    pose_result.orientation.y = ang[1] 
    pose_result.orientation.z = ang[2] 
    pose_result.orientation.w = ang[3]
    return pose_result

def pose_to_frame(pose):
    frame_result = PyKDL.Frame() 
    frame_result.p = PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z) 
    frame_result.M = PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    return frame_result

def pose_to_poseStamped(pose):
    poseSt = PoseStamped()
    poseSt.pose=pose
    poseSt.header.stamp=rospy.get_rostime()
    poseSt.header.frame_id="base_link"
    return poseSt

def get_shifted_pose_abs(origin_pose, shift):
    origin_pose.position.x += shift[0]
    origin_pose.position.y += shift[1]
    origin_pose.position.z += shift[2]
    tf_origin = pose_to_frame(origin_pose)   
    tf_origin.M.DoRotZ(shift[5]) 
    tf_origin.M.DoRotY(shift[4]) 
    tf_origin.M.DoRotX(shift[3])
    pose_result = frame_to_pose(tf_origin)
    return pose_result

def get_shifted_pose_rel(origin_pose, shift):
    tf_origin = pose_to_frame(origin_pose)   
    tf_shift = PyKDL.Frame() 
    tf_shift.p.x = PyKDL.Vector(shift[0], shift[1], shift[2]) 
    tf_shift.M.DoRotX(shift[3]) 
    tf_shift.M.DoRotY(shift[4]) 
    tf_shift.M.DoRotZ(shift[5])
    tf_result = tf_origin * tf_shift
    pose_result = frame_to_pose(tf_result)
    return pose_result

def visualize_keypoints_simple(poses, ref_frame = "/base_link"):
        while not rospy.is_shutdown():
                number = 2000
                for waypoint in poses:
                        current_time = rospy.get_rostime()
                        waypoint_frame = pose_to_frame(waypoint)
                        broadcastTransform(br, waypoint_frame, str(number), ref_frame, time=current_time)
                        number += 1

#Services
def get_moveit_groups_service_callback(req): 
    """
    Launches a launch file
    """
    global to_launch
    resp = MoveitGroupsResponse()
    try:
        groups = robot.get_group_names()
        for group in groups:
            msg_i = GroupTargets()
            msg_i.group = group
            named_targets = moveit_groups[group].get_named_targets()
            for target in named_targets:
                msg_i.targets.append(target)
            resp.groups.append(msg_i)
        resp.success = True
    except:
        resp.success = False
    return resp

rospy.Service(get_moveit_groups_service, MoveitGroups, get_moveit_groups_service_callback)


def move_group_service_callback(req): 
    """
    Launches a launch file
    """
    global to_launch
    resp = MoveGroupSrvResponse()
    for group in moveit_groups:
        moveit_groups[group].clear_pose_targets()
    try:
        if req.type==0:
            moveit_groups[req.group].set_named_target(req.target_named)
        elif req.type==1: #relative
            poseL=moveit_groups["arm_left"].get_current_pose().pose
            print(poseL)
            DL=req.target_pose.left
            displacementL=[(DL.x)/1000, (DL.y)/1000, (DL.z)/1000, (DL.rx)*(math.pi/180), (DL.ry)*(math.pi/180), (DL.rz)*(math.pi/180)]
            target_poseL=get_shifted_pose_abs(copy.deepcopy(poseL),displacementL)
            print(target_poseL)
            moveit_groups["arm_left"].set_pose_target(pose_to_poseStamped(target_poseL))
            moveit_groups["arms"].set_pose_target(pose_to_poseStamped(target_poseL), "arm_left_link_7_t")
            poseR=moveit_groups["arm_right"].get_current_pose().pose
            DR=req.target_pose.right
            displacementR=[(DR.x)/1000, (DR.y)/1000, (DR.z)/1000, (DR.rx)*(math.pi/180), (DR.ry)*(math.pi/180), (DR.rz)*(math.pi/180)]
            target_poseR=get_shifted_pose_abs(copy.deepcopy(poseR),displacementR)
            moveit_groups["arm_right"].set_pose_target(pose_to_poseStamped(target_poseR))
            moveit_groups["arms"].set_pose_target(pose_to_poseStamped(target_poseR), "arm_right_link_7_t")
            #visualize_keypoints_simple([poseL, target_poseL])
        elif req.type==2: #absolute
            poseL=Pose()
            poseL.orientation.w = 1
            DL=req.target_pose.left
            displacementL=[(DL.x)/1000, (DL.y)/1000, (DL.z)/1000, (DL.rx)*(math.pi/180), (DL.ry)*(math.pi/180), (DL.rz)*(math.pi/180)]
            #displacementL=[(DL.x)/1000, (DL.y)/1000, (DL.z)/1000, 0,0,0]
            target_poseL=get_shifted_pose_abs(poseL,displacementL)
            moveit_groups["arm_left"].set_pose_target(target_poseL)
            moveit_groups["arms"].set_pose_target(pose_to_poseStamped(target_poseL), "arm_left_link_7_t")
            poseR=Pose()
            poseR.orientation.w = 1
            DR=req.target_pose.right
            displacementR=[(DR.x)/1000, (DR.y)/1000, (DR.z)/1000, (DR.rx)*(math.pi/180), (DR.ry)*(math.pi/180), (DR.rz)*(math.pi/180)]
            #displacementR=[(DR.x)/1000, (DR.y)/1000, (DR.z)/1000, 0,0,0]
            target_poseR=get_shifted_pose_abs(poseR,displacementR)
            moveit_groups["arm_right"].set_pose_target(target_poseR)
            moveit_groups["arms"].set_pose_target(pose_to_poseStamped(target_poseR), "arm_right_link_7_t")
            #visualize_keypoints_simple([target_poseL, target_poseR])
        else:
            resp.success = False
            return resp
        moveit_groups[req.group].go(wait=True)
        resp.success = True
        for group in moveit_groups:
            moveit_groups[group].clear_pose_targets()
    except:
        resp.success = False
        for group in moveit_groups:
            moveit_groups[group].clear_pose_targets()
    return resp

rospy.Service(move_group_service, MoveGroupSrv, move_group_service_callback)


def get_arms_pose_callback(req): 
    """
    Launches a launch file
    """
    global to_launch
    resp = ArmsPoseResponse()
    try:
        poseL=moveit_groups["arm_left"].get_current_pose().pose
        frameL = pose_to_frame(poseL)
        resp.left.x = frameL.p[0]
        resp.left.y = frameL.p[1]
        resp.left.z = frameL.p[2]
        anglesL = frameL.M.GetEulerZYX()
        resp.left.rz = anglesL[0]
        resp.left.ry = anglesL[1]
        resp.left.rx = anglesL[2]
        poseR=moveit_groups["arm_right"].get_current_pose().pose
        frameR = pose_to_frame(poseR)
        resp.right.x = frameR.p[0]
        resp.right.y = frameR.p[1]
        resp.right.z = frameR.p[2]
        anglesR = frameR.M.GetEulerZYX()
        resp.right.rz = anglesR[0]
        resp.right.ry = anglesR[1]
        resp.right.rx = anglesR[2]
        resp.success = True
    except:
        resp.success = False
    return resp

rospy.Service(get_arms_pose_service, ArmsPose, get_arms_pose_callback)


rospy.spin()