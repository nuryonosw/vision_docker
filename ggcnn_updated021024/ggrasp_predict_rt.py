#! /usr/bin/env python3

import time
import os
import rospy

import numpy as np
import torch

import cv2
import scipy.ndimage as ndimage
from skimage.draw import circle, line
from skimage.feature import peak_local_max

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray

from ggrasp.ggrasp import predict
from dougsm_helpers.timeit import TimeIt

bridge = CvBridge()

rospy.init_node('ggrasp_detection')

# Output publishers.
grasp_pub = rospy.Publisher('ggrasp/img/grasp', Image, queue_size=1)
grasp_plain_pub = rospy.Publisher('ggrasp/img/grasp_plain', Image, queue_size=1)
depth_pub = rospy.Publisher('ggrasp/img/depth', Image, queue_size=1)
ang_pub = rospy.Publisher('ggrasp/img/ang', Image, queue_size=1)
cmd_pub = rospy.Publisher('ggrasp/out/command', Float32MultiArray, queue_size=1)

# Initialise some globals.
prev_mp = np.array([150, 150])

# Get the camera parameters
camera_info_msg = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo)
K = camera_info_msg.K
fx = K[0]
cx = K[2]
fy = K[4]
cy = K[5]

here = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(here, "..", "saved_models", "ggcnn_jacquard_smooth_0.5_iou_0.77") #it was ggcnn
model = torch.load(model_path)
model.to(torch.device("cuda"))

def depth_callback(depth_message):
    global prev_mp
    global fx, cx, fy, cy

    with TimeIt('Predict'):
        depth = bridge.imgmsg_to_cv2(depth_message)
        
        #  Crop a square out of the middle of the depth and resize it to 300*300
        crop_size = 300   #changed from 300 on Oct 02 2024 1501pm
        crop_offset = 40
        out_size = 300
        
        points_out, ang_out, width_out, depth_crop = predict(depth, model, crop_size=crop_size, out_size=out_size, crop_y_offset=crop_offset, filters=(2.0, 2.0, 2.0))

    with TimeIt('Calculate Depth'):
        # Figure out roughly the depth in mm of the part between the grippers for collision avoidance.
        depth_center = depth_crop[100:141, 130:171].flatten()
        depth_center.sort()
        depth_center = depth_center[:10].mean() * 1000.0

    with TimeIt('Control'):
        # Calculate the best pose from the camera intrinsics.
        maxes = None
        ALWAYS_MAX = False  # Use ALWAYS_MAX = True for the open-loop solution. changed to False 12 nov 2023 15:57

        if ALWAYS_MAX:
            # Track the global max.
            max_pixel = np.array(np.unravel_index(np.argmax(points_out), points_out.shape))
            prev_mp = max_pixel.astype(np.int)
        else:
            # Calculate a set of local maxes.  Choose the one that is closes to the previous one.# it was threshold 0.1
            maxes = peak_local_max(points_out, min_distance=10, threshold_abs=0.1, num_peaks=3)
            if maxes.shape[0] == 0:
                rospy.logerr('No Local Maxes')
                return
            max_pixel = maxes[np.argmin(np.linalg.norm(maxes - prev_mp, axis=1))]

            # Keep a global copy for next iteration.
            prev_mp = (max_pixel * 0.25 + prev_mp * 0.75).astype(np.int)

        ang = ang_out[max_pixel[0], max_pixel[1]]
        width = width_out[max_pixel[0], max_pixel[1]]

        # Convert max_pixel back to uncropped/resized image coordinates in order to do the camera transform.
        max_pixel = ((np.array(max_pixel) / out_size * crop_size) + np.array([(480 - crop_size)//2 - crop_offset, (640 - crop_size) // 2]))
        max_pixel = np.round(max_pixel).astype(np.int)

        point_depth = depth[max_pixel[0], max_pixel[1]]

        # Compute the actual position.
        x = (max_pixel[1] - cx)/(fx) * point_depth
        y = (max_pixel[0] - cy)/(fy) * point_depth
        z = point_depth

        if np.isnan(z):
            return

    with TimeIt('Draw'):
        # Draw grasp markers on the points_out and publish it. (for visualisation)
        grasp_img = cv2.applyColorMap((points_out * 255).astype(np.uint8), cv2.COLORMAP_JET)
       
        grasp_img_plain = grasp_img.copy()

        rr, cc = circle(prev_mp[0], prev_mp[1], 5)
        grasp_img[rr, cc, 0] = 0
        grasp_img[rr, cc, 1] = 255
        grasp_img[rr, cc, 2] = 0
        cv2.putText(grasp_img_plain, "Q={}".format(max_pixel), (20, 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
        
        ang_img=cv2.applyColorMap((ang_out * 255).astype(np.uint8), cv2.COLORMAP_JET)
    with TimeIt('Publish'):
        # Publish the output images (not used for control, only visualisation)
        grasp_img = bridge.cv2_to_imgmsg(grasp_img, 'bgr8')
        grasp_img.header = depth_message.header
        grasp_pub.publish(grasp_img)

        grasp_img_plain = bridge.cv2_to_imgmsg(grasp_img_plain, 'bgr8')
        grasp_img_plain.header = depth_message.header
        grasp_plain_pub.publish(grasp_img_plain)

        depth_pub.publish(bridge.cv2_to_imgmsg(depth_crop, encoding=depth_message.encoding))
        
        ang_img=bridge.cv2_to_imgmsg(ang_out)
        #ang_pub.publish(bridge.cv2_to_imgmsg(ang_out,'64FC1'))
        ang_pub.publish(ang_img)

        # Output the best grasp pose relative to camera.
        cmd_msg = Float32MultiArray()
        cmd_msg.data = [x, y, z, ang, width, depth_center]
        cmd_pub.publish(cmd_msg)


depth_sub = rospy.Subscriber('/camera/depth/image_meters', Image, depth_callback, queue_size=1)

while not rospy.is_shutdown():
    rospy.spin()
