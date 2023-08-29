#!/usr/bin/env python3

import sys, math, copy
import rospy, tf, geometry_msgs.msg

from moveit_commander import MoveGroupCommander, RobotCommander
from geometry_msgs.msg import Pose, PoseStamped

if __name__ == '__main__':
    
    node_name = "commander_example"
    rospy.init_node( node_name, anonymous=True )
    
    group = MoveGroupCommander("arm")
    
    group.set_planning_time( 100.0 )
    
    # Getting Initial Pose & RPY
    pose_init = group.get_current_pose()
    rospy.loginfo( "Get Initial Pose\n{}".format( pose_init ) )
    rpy_init  = group.get_current_rpy()
    rospy.loginfo( "Get Initial RPY:{}".format( rpy_init ) )
    
    # Pose 1
    rospy.loginfo( "Starting Pose 1")
    pose_target_1 =  [ 0.12, 0.0, 0.1, 0.0, math.pi/2.0, 0.0 ] # [ x, y, z, r, p, y ]
    group.set_pose_target( pose_target_1 )
    group.go()
    
    rospy.sleep(2.0)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )
    
    # Pose 2
    rospy.loginfo( "Starting Pose 2")
    group.set_pose_target( [ 0.2, 0.0, 0.2, 0.0, 0.0, 0.0 ] )
    group.go()
    
    rospy.sleep(2.0)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )
    
    # Pose 3
    rospy.loginfo( "Starting Pose 3")
    
    pose_target_3 = Pose()
    pose_target_3.position.x =  0.10
    pose_target_3.position.y =  0.10
    pose_target_3.position.z =  0.10
    pose_target_3.orientation.x = -0.2706
    pose_target_3.orientation.y =  0.6533
    pose_target_3.orientation.z =  0.2706
    pose_target_3.orientation.w =  0.6533
    
    group.set_joint_value_target( pose_target_3, True )
    group.go()
    
    rospy.sleep(2.0)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )
    
    # Pose 3 Z:-0.05[m]
    rospy.loginfo( "Starting Pose 3 Z:-0.05[m]")
    pose_target_3.position.z += 0.05
    
    group.set_joint_value_target( pose_target_3, True )
    group.go()
    
    rospy.sleep(2.0)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )
    
    # Pose 4
    rospy.loginfo( "Starting Pose 4")
    
    pose_target_4 = Pose()
    pose_target_4.position.x =  0.10
    pose_target_4.position.y = -0.10
    pose_target_4.position.z =  0.05
    pose_target_4.orientation.x =  0.2706
    pose_target_4.orientation.y =  0.6533
    pose_target_4.orientation.z = -0.2706
    pose_target_4.orientation.w =  0.6533
    
    group.set_joint_value_target( pose_target_4, True )
    group.go()
    
    rospy.sleep(2.0)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )
    
    # Pose 4 Z:+0.05[m]
    rospy.loginfo( "Starting Pose 4 Z:+0.05[m]")
    pose_target_4.position.z += 0.05
    
    group.set_joint_value_target( pose_target_4, True )
    group.go()
    
    rospy.sleep(2.0)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )
    
    # Back to Initial Pose
    rospy.loginfo( "Back to Initial Pose")
    group.set_joint_value_target( pose_init, True )
    group.go()
        
    rospy.sleep(2.0)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )



    # Pose 5 Z:+0.15[m]
    rospy.loginfo( "Starting Pose 5 Z:+0.05[m]")
    pose_target_4.position.z += 0.05
    
    group.set_joint_value_target( pose_target_4, True )
    group.go()
    
    rospy.sleep(2.0)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )
