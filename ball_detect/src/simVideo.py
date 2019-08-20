#! /usr/bin/python
# coding:utf-8

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import cv2
import os
import numpy as np
import glob
from cv_bridge import CvBridge, CvBridgeError


def publishImage():
	rospy.init_node('simulatVideo', anonymous=True)
	img_pub = rospy.Publisher('/simulationVideo/image_raw', Image, queue_size = 2)
	rate = rospy.Rate(10)
	videoDir = "/home/zmart/video"
	bridge = CvBridge()
        img = cv2.imread('/home/zmart/darknet/data/1.jpg')
	'''
	while not rospy.is_shutdown():
		msg = bridge.cv2_to_imgmsg(img, encoding = "bgr8")
		img_pub.publish(msg)
		rate.sleep()
	'''
	while not rospy.is_shutdown():
		for fn in glob.glob(videoDir + "/*.mp4"):
			print("fn = ", fn)
			cap = cv2.VideoCapture(fn)
			if not cap.isOpened():
				sys.stdout.write("fuck, can't open the video")
				return -1
			cnt = 0
			totalframes = cap.get(7)
			while(cnt < totalframes):
				ret, frame = cap.read()
				try:
					frame = np.rot90(frame)
					msg = bridge.cv2_to_imgmsg(frame, encoding = "bgr8")
					img_pub.publish(msg)
				except:
					pass
				cnt = cnt + 1
		rate.sleep()
	

if __name__ == '__main__':
	try:
		publishImage()
	except rospy.ROSInterruptException:
		pass
