#! /usr/bin/env python3

import rospy
import tf.transformations as tft

import numpy as np


import std_msgs.msg
import std_srvs.srv
import geometry_msgs.msg


import sys
import copy
import math

import moveit_commander
import moveit_msgs.msg
from tf.transformations import quaternion_from_euler

import dougsm_helpers.tf_helpers as tfh


from moveit_commander import MoveGroupCommander, RobotCommander
from geometry_msgs.msg import Pose, PoseStamped

#from ar_track_alvar_msgs.msg import AlvarMarkers
#from ar_track_alvar_msgs.msg import AlvarMarker
from std_msgs.msg import Float32MultiArray

def move_home():
	arm_group.set_named_target("home")
	print ("Executing Move: Home")
	#plan1 = arm_group.plan()
	plan_success, plan1, planning_time, error_code = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(1)

def move_lookdown():
	arm_group.set_named_target("look_down")
	print ("Executing Move: look_down")
	#plan1 = arm_group.plan()
	plan_success, plan1, planning_time, error_code = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(1)



def execute_grasp():
    # Execute a grasp.
    global MOVING
    global CURR_Z
    global start_force_srv
    global stop_force_srv
    msg = rospy.wait_for_message("ggrasp/out/command", Float32MultiArray)
    if msg.data:
            gp = geometry_msgs.msg.Pose()
            gp.position.x = msg.data[0]
            gp.position.y = msg.data[1]
            gp.position.z = msg.data[2]
            gp.orientation.w = 1
    #print("Posisinya",marker_position)
            print("grasp terlihat")
            gripper_group.set_joint_value_target([0.7, 0.7])
            gripper_group.go()
            target_pose=tfh.convert_pose(gp,'camera_depth_optical_frame','world')
            print('target \n',target_pose)
            rospy.sleep(5)          
            target_pose.position.y = (target_pose.position.y) #added
            target_pose.position.z = target_pose.position.z+0.1-0.03
            #q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
            #q = quaternion_from_euler( 0.0, math.pi/2.0, 0.0)
            #q = quaternion_from_euler( math.pi, 0.0, msg.data[3]) #added for orientation Nov 27 2023 10.50
            #orientation_list = [ox,oy,oz,ow]
            #eu = euler_from_quaternion(orientation_list)
            #q = quaternion_from_euler( 0.0,  math.radians(180), math.radians(-90) )
            q = quaternion_from_euler( 0.0,  math.radians(180), (msg.data[3]-math.radians(+90)) )
            #q = quaternion_from_euler( 0.0, math.pi/2.0, msg.data[3])
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            arm_group.set_pose_target( target_pose )	# Target 
            arm_group.go()
    else :
            print("Objek tidak terlihat") 
            

    return


if __name__ == '__main__':
    rospy.init_node('ggcnn_follower')
    ###### Setup ########
    moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node('move_group_python_execute_trajectory', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm_group = moveit_commander.MoveGroupCommander("arm")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

    #Had probelms with planner failing, Using this planner now. I believe default is OMPL
    arm_group.set_planner_id("RRTConnectkConfigDefault")
    #Increased available planning time from 5 to 10 seconds--changed to 2 secs nov 4 2024 adn remove ;
    arm_group.set_planning_time(2)
    
    # Home position.
    move_home()

    while not rospy.is_shutdown():

        rospy.sleep(2.5)
        
        #input('Press Enter to Start.')
        
        move_lookdown()
        # start_record_srv(std_srvs.srv.TriggerRequest())
        rospy.sleep(2.5)
        #input('Press Enter to find')
        #rospy.sleep(0.5)
        execute_grasp()
        #marker_position = msg.markers[0].pose.pose
        gripper_group.set_joint_value_target((0.005, 0.0005))
        gripper_group.go()
        rospy.sleep(0.5)
        # stop_record_srv(std_srvs.srv.TriggerRequest())

        #input('Press Enter to Complete')
        move_home()
        rospy.sleep(0.5)
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.45
        target_pose.position.y = 0.0
        target_pose.position.z = 0.2
        #q = quaternion_from_euler( 0.0,  math.radians(180), 0.0 )
        #q = quaternion_from_euler( 0.0,  math.radians(180), math.radians(-90) )
        #q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        #target_pose.orientation.x = q[0]
        #target_pose.orientation.y = q[1]
        #target_pose.orientation.z = q[2]
        #target_pose.orientation.w = q[3]
        target_pose.orientation.x = 0.4599710017377993
        target_pose.orientation.y = 0.4224692814906936
        target_pose.orientation.z = 0.5596712798263089
        target_pose.orientation.w = 0.5447150101609501
        arm_group.set_pose_target(target_pose)  # 目標ポーズ設定
        arm_group.go()  # 実行
        rospy.sleep(0.5)
        gripper_group.set_joint_value_target([0.5, 0.5])
        gripper_group.go()
        rospy.sleep(0.5)
       
        gripper_group.set_joint_value_target([0.005, 0.005])
        gripper_group.go()
        move_home()