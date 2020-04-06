from collections import Counter
import rospy
import time
from sensor_msgs.msg import Image

from itertools import product
import scipy.stats
from scipy.sparse import coo_matrix, dia_matrix
from scipy.sparse import linalg
#from skimage import io
import time

import cv2
import numpy as np
import sys
#from kinect_smoothing import HoleFilling_Filter, Denoising_Filter
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
Depth_image = None
RGB_image = None
flag = None
cheight = None
cwidth = None


def callbackDepth(data):
	global Depth_image
	global flag
	Depth_image = bridge.imgmsg_to_cv2(data, desired_encoding='32UC1')
	#Depth_image = cv2.inRange(Depth_image, 0, 1000)
	#Depth_image = np.asarray(Depth_image)
	#Depth_image = hole_filter.smooth_image_frames(Depth_image)
	#Depth_image = cv2.GaussianBlur(Depth_image,(5,5),0)
	#Depth_image = np.uint8(Depth_image)
	#Depth_image = cv2.normalize(Depth_image, 0, 255, cv2.NORM_MINMAX)
	#Depth_image = cv2.fastNlMeansDenoising(Depth_image)
	#row, col = Depth_image.shape
	#for r in xrange(row):
    	#	for c in xrange(col):
	#		px = Depth_image[r][c]
	#		if px == 0:
	#			Depth_image[r][c] = 255
	print(Depth_image[294][388])
	flag    =   1
	
def callbackRGB(data):
	global RGB_image
	RGB_image = bridge.imgmsg_to_cv2(data, data.encoding)

# print(cv_image)
# print(image.encoding)
# (rows, cols, channels) = cv_image.shape
# new_image = cv_image
# return cv_image
# cv2.waitKey(3	while(1):)
#cv2.meanStdDev

if __name__ == '__main__':

	#hole_filter = HoleFilling_Filter(flag='min')
	rospy.init_node('Backgroun_Subtract')
	rospy.Subscriber('/kinect2/qhd/image_color', Image, callbackRGB)
	rospy.Subscriber('/kinect2/sd/image_depth', Image, callbackDepth)
	#rospy.Subscriber('/kinect2/qhd/image_depth_rect', Image, callbackDepth)
	rate = 100
	r = rospy.Rate(rate)
	while Depth_image is None:
		print("Waiting for depth image")
		r.sleep()
	while RGB_image is None:
		print("Waiting for color image")
		r.sleep()
	#first_frame = cv2.imread('background_front.jpg',0)

	histry = 10
	fgbg = cv2.createBackgroundSubtractorMOG2(history = histry, detectShadows = False)
	for x in range(histry):
		flag = 0
		print("Sample " + str(x) + "!")
		while flag != 1:
			None
		#Depth_image = fill_depth_colorization(RGB_image, Depth_image)
		fgmask = fgbg.apply(Depth_image)

		#lower_black = np.array(126)
		#upper_black = np.array(130)
	while True:
		#Depth_image = fill_depth_colorization(RGB_image, Depth_image)
		fgmask = fgbg.apply(Depth_image, learningRate = 0)
		#fgmaskGr = cv2.inRange(fgmask, lower_black, upper_black)
		#fgmask = cv2.inRange(fgmask, 254, 1)
		cv2.imshow("Color Kinect", RGB_image)
		cv2.imshow("Depth Kinect", Depth_image)
		cv2.imshow("DepthSubtract", fgmask)
		#cv2.imshow("DepthSubtractGR", fgmaskGr)
		if cv2.waitKey(1) == 27:
			break  # esc to quit
	cv2.destroyAllWindows()
