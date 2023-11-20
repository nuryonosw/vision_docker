#!/usr/bin/env python3
import os
import sys, math, copy
import rospy, tf, geometry_msgs.msg,std_msgs
#import tf2
from moveit_commander import MoveGroupCommander, RobotCommander
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float32MultiArray

#from ar_track_alvar_msgs.msg import AlvarMarkers
#from ar_track_alvar_msgs.msg import AlvarMarker
import dougsm_helpers.tf_helpers as tfh
#from helpers.transforms import current_robot_pose, publish_tf_quaterion_as_transform, convert_pose, publish_pose_as_transform





def callback(grsp_cmd):
    print('Callback Grasp Command')
   
    #rospy.loginfo("I heard %s", grsp_cmd.data [1])
    print("Posisi X(m)",grsp_cmd.data [0])
    print("Posisi Y(m)",grsp_cmd.data [1])
    print("Posisi Z(m)",grsp_cmd.data [2])
    print("Sudut grasp",grsp_cmd.data[3])
    print("Lebar grasp (pixel)",grsp_cmd.data[4])
    print("Depth Center",grsp_cmd.data[5])
    gp = geometry_msgs.msg.Pose()
    gp.position.x = grsp_cmd.data[0]
    gp.position.y = grsp_cmd.data[1]
    gp.position.z = grsp_cmd.data[2]
    #gp.position.x = 0
    #gp.position.y = 0
    #gp.position.z = 0
    gp.orientation.w = 1
    #print(gp)
    
    #gp_base = tfh.convert_pose(gp, 'camera_link', 'world')
    gp_base=tfh.convert_pose(gp,'camera_depth_optical_frame','world')
    print('target \n',gp_base)
    #print("datanya",markers.markers[0].pose.pose)
    
    

        
def grasp_listener():
    rospy.init_node('grasp_cmd_data', anonymous=True)
      
     
    
    rospy.Subscriber("ggrasp/out/command", Float32MultiArray, callback)
    rospy.spin()


if __name__ == '__main__':
    print('Grasp command listener started .......')
    #print(['seq', 'time_stamp', 'frame_id', 'pos_x', 'pos_y', 'pos_z', 'roll_x', 'pitch_y', 'yaw_z'])
    
    
    
    grasp_listener()
    
    
    