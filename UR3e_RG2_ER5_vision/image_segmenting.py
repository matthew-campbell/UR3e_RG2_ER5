from collections import Counter
import rospy
import time
from sensor_msgs.msg import Image

import time

import cv2
import numpy as np
import sys
import math

#from kinect_smoothing import HoleFilling_Filter, Denoising_Filter
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
Depth_image = None
Bksub_image = None
RGB_image = None
flag = None
cheight = None
cwidth = None




def callbackDepth(data):
	global Depth_image
	global flag
	Depth_image2 = np.asarray(bridge.imgmsg_to_cv2(data, 'passthrough'))
	Depth_image = np.nan_to_num(Depth_image2)

def callbackBKSub(data):
	global Depth_image
	global Bksub_image
	global flag
	Bksub_image = np.asarray(bridge.imgmsg_to_cv2(data, 'passthrough'))
	flag    =   1
	
def callbackRGB(data):
	global RGB_image
	RGB_image = bridge.imgmsg_to_cv2(data, data.encoding)


if __name__ == '__main__':

	rospy.init_node('segmentation')
	rospy.Subscriber('/kinect2/qhd/image_color', Image, callbackRGB)
	rospy.Subscriber('bksub_image', Image, callbackBKSub)
	rospy.Subscriber('/kinect2/sd/image_depth', Image, callbackDepth)
	
	#PubDepthImg = rospy.Publisher('depth_image_norm', String)
	PubBksubImg = rospy.Publisher('bksub_image', Image)
	
	rate = 100
	r = rospy.Rate(rate)
	while Depth_image is None:
		print("Waiting for depth image")
		r.sleep()
	while RGB_image is None:
		print("Waiting for color image")
		r.sleep()

	print("Found it!")
	while True:
		
		cv2.imshow("Color Kinect", RGB_image)

		ret,thresh = cv2.threshold(Bksub_image,127,255,0)
		# find contours in the binary image
		im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		for c in contours:
	   	# calculate moments for each contour
	   		M = cv2.moments(c)
			print("Moment:")
			print(M)
			print("Contuor:")
			print(c)
		#depth_message = bridge.cv2_to_imgmsg(Depth_image, encoding="passthrough")
		try:		
			None
		except:
			#print("Not Published")
			None
		
		if cv2.waitKey(1) == 27:
			break  # esc to quit
	cv2.destroyAllWindows()
