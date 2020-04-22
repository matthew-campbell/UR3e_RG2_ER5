#!/usr/bin/env python
from collections import Counter
import rospy
import time
from sensor_msgs.msg import Image

import time

import cv2
import numpy as np
import sys
import math

import random as rng

#from kinect_smoothing import HoleFilling_Filter, Denoising_Filter
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
Depth_image = None
Bksub_image = None
RGB_image = None
flag = None
cheight = None
cwidth = None
pName = "image_segmentation: "



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
	PubBksubImg = rospy.Publisher('bksub_image', Image, queue_size=10)
	
	rate = 100
	r = rospy.Rate(rate)
	while Depth_image is None:
		print(pName + "Waiting for depth image")
		r.sleep()
	while RGB_image is None:
		print(pName + "Waiting for color image")
		r.sleep()

	print(pName + "Found it!")
	color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
	while True:
		
		cv2.imshow(pName + "Color Kinect", RGB_image)
		cv2.imshow(pName + "Depth Kinect", Depth_image)

		ret,thresh = cv2.threshold(Bksub_image,127,255,0)
		# find contours in the binary image
		im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		
		centers = [None]*len(contours)
		radius = [None]*len(contours)
		mu = [None]*len(contours)
		mc = [None]*len(contours)
		max_depth = [0.0]*len(contours)
		min_depth = [1000.0]*len(contours)
		mid_depth = [0.0]*len(contours)
		
		for i in range(len(contours)):
			print("The min_depth is " + str(min_depth[0]))
			mu[i] = cv2.moments(contours[i])
			# Get the mass centers
			centers[i], radius[i] = cv2.minEnclosingCircle(contours[i])
			
			for j in range(len(contours[i])):
				try:
					x = Depth_image[contours[i][j][0][0],contours[i][j][0][1]]
					print("J is"+ str(j) + "x is:" + str(x))
				except:
					None
	
				if x <= min_depth[i] and not 0:
					min_depth[i] = x
				if x >= max_depth[i] and not 0:
					max_depth[i] = x
			print((max_depth[i] - min_depth[i])/2.0)
		for i in range(len(contours)):
			# add 1e-5 to avoid division by zero
			mc[i] = (mu[i]['m10'] / (mu[i]['m00'] + 1e-5), mu[i]['m01'] / (mu[i]['m00'] + 1e-5))
			
		for i in range(len(contours)):
			cv2.circle(RGB_image, (int(mc[i][0]), int(mc[i][1])), int(radius[i]), color, 2)
			cv2.drawContours(RGB_image, contours, i, color, 2)
			cv2.circle(RGB_image, (int(mc[i][0]), int(mc[i][1])), 4, color, -1)

		#depth_message = bridge.cv2_to_imgmsg(Depth_image, encoding="passthrough")
		try:		
			None
		except:
			#print("Not Published")
			None
		
		if cv2.waitKey(1) == 27:
			break  # esc to quit
	cv2.destroyAllWindows()
