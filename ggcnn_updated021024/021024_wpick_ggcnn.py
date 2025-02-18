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

from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
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

def move_zero():
	arm_group.set_named_target("zero")
	print ("Executing Move: Zero")
	#plan1 = arm_group.plan()
	plan_success, plan1, planning_time, error_code = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(1)

def move_sleep():
	arm_group.set_named_target("sleep")
	print ("Executing Move: Sleep")
	#plan1 = arm_group.plan()
	plan_success, plan1, planning_time, error_code = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(1)

def move_reach_out():
	arm_group.set_named_target("reach_out")
	print ("Executing Move: Reach out")
	#plan1 = arm_group.plan()
	plan_success, plan1, planning_time, error_code = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(1)
    
def gripper_close():
    gripper_group_variable_values = gripper_group.get_current_joint_values()
    gripper_group_variable_values[0] = 00.001 # close empty claws 00.005
    gripper_group.set_joint_value_target(gripper_group_variable_values)
    gripper_group.go()

def gripper_open():
    gripper_group_variable_values = gripper_group.get_current_joint_values()
    gripper_group_variable_values[0] = -00.003 # close empty claws 00.005
    gripper_group.set_joint_value_target(gripper_group_variable_values)
    gripper_group.go()

def gripper_widden(lebarnya):
    gripper_group_variable_values = gripper_group.get_current_joint_values()
    gripper_group_variable_values[0] = lebarnya # close empty claws 00.005
    gripper_group.set_joint_value_target(gripper_group_variable_values)
    gripper_group.go()

def lower_z(turunnya):
    variable = arm_group.get_current_pose()
    print (variable.pose)
    variable.pose.position.z= variable.pose.position.z-turunnya
    arm_group.set_pose_target( variable.pose)
    arm_group.go()


def longer_x():
    variable = arm_group.get_current_pose()
    print (variable.pose)
    variable.pose.position.x= variable.pose.position.x+0.15
    arm_group.set_pose_target( variable.pose)
    arm_group.go()

def execute_grasp():
    # Execute a grasp.
    global MOVING
    global CURR_Z
    global start_force_srv
    global stop_force_srv
    msg = rospy.wait_for_message("ggrasp/out/command", Float32MultiArray)
    if msg.data:
            
            cur_pose = arm_group.get_current_pose()
            CURR_Z=cur_pose.pose.position.z
            print('EE Z_POS',cur_pose.pose.position.z)
            gp = geometry_msgs.msg.Pose()
            gp.position.x = msg.data[0]
            gp.position.y = msg.data[1]
            gp.position.z = msg.data[2]
            gp.orientation.w = 1
            grip_width = msg.data[4]
    # Convert width in pixels to mm.
    # 0.07 is distance from end effector (CURR_Z) to camera.
    # 0.1 is approx degrees per pixel for the realsense.
            g_width = 2 * ((CURR_Z + 0.07)) * np.tan(0.1 * grip_width / 2.0 / 180.0 * np.pi) * 1000
    #print("Posisinya",marker_position)
            print("grasp terlihat")
            target_pose=tfh.convert_pose(gp,'camera_depth_optical_frame','world')
            print('target \n',target_pose)
            rospy.sleep(5)          
            target_pose.position.y = (target_pose.position.y) #added---removed -0.0175
            target_pose.position.z = target_pose.position.z+0.05
            #q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
            #q = quaternion_from_euler( 0.0, math.pi/2.0, 0.0)
            #q = quaternion_from_euler( math.pi, 0.0, msg.data[3]) #added for orientation Nov 27 2023 10.50
            #orientation_list = [ox,oy,oz,ow]
            #eu = euler_from_quaternion(orientation_list)
            q = quaternion_from_euler( 0.0, math.pi/2.0, msg.data[3])
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            arm_group.set_pose_target( target_pose )	# Target 
            arm_group.go()
            print(g_width)
            print("Fully Open Gripper")
            gripper_open()
            print("Move Down")
            #target_pose.position.x = x
            #target_pose.position.y = y
            target_pose.position.z = target_pose.position.z-0.05 #it was 0.15 when claws
            arm_group.set_pose_target( target_pose )	# Target 
            arm_group.go()
            gripper_group_variable_values = gripper_group.get_current_joint_values()
            gripper_group_variable_values[0] = 00.0015 #it was 00.0015 when claws
            gripper_group.set_joint_value_target(gripper_group_variable_values)
            gripper_group.go()
    else :
            print("Objek tidak terlihat") 
            

    return


if __name__ == '__main__':
    rospy.init_node('ar_follower')
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
    #Increased available planning time from 5 to 10 seconds
    arm_group.set_planning_time(10);
    
    # Home position.
    move_home()
    gripper_close()

    while not rospy.is_shutdown():
        gripper_close()
        rospy.sleep(0.5)
        
        input('Press Enter to Start.')
        
        move_sleep()
        # start_record_srv(std_srvs.srv.TriggerRequest())
        rospy.sleep(0.5)
        lower_z(0.15)
        rospy.sleep(0.5)
        
        input('Press Enter to find')
        #rospy.sleep(0.5)
        execute_grasp()
        #marker_position = msg.markers[0].pose.pose
        
        rospy.sleep(0.5)
        # stop_record_srv(std_srvs.srv.TriggerRequest())
        
        input('Press Enter to Complete')
        
        move_sleep()
        rospy.sleep(0.5)
        move_reach_out()
        rospy.sleep(0.5)
        print("Fully Open Gripper")
        #gripper_open()
        gripper_widden(-0.003)
        rospy.sleep(1)