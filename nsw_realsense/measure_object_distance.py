#https://pysource.com
import cv2
from realsense_camera import *
from mask_rcnn import *

# Load Realsense camera
rs = RealsenseCamera()
mrcnn = MaskRCNN()

while True:
	# Get frame in real time from Realsense camera
	ret, rgb_frame, depth_frame,ir_frame = rs.get_frame_stream()

	# Get object mask
	boxes, classes, contours, centers = mrcnn.detect_objects_mask(rgb_frame)

	# Draw object mask
	rgb_frame = mrcnn.draw_object_mask(rgb_frame)

	# Show depth info of the objects
	mrcnn.draw_object_info(rgb_frame, depth_frame)


	cv2.imshow("depth frame", depth_frame)
	cv2.imshow("RGB frame", rgb_frame)

	key = cv2.waitKey(1)
	if key == 27:
		break

rs.release()
cv2.destroyAllWindows()
