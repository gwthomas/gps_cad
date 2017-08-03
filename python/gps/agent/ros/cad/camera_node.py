#!/usr/bin/env python

##	Code taken from the @package click_window
#	This module uses OpenCV's HighGUI platform to import a camera stream and allow a
#	person to click on an arbitrary pixel at arbitrary levels of zoom. It outputs
#	a message containing the pixel value (at zoom 100%) and camera_info, to the
#	topic specified by "outputName"

import numpy as np
import roslib
import sys
roslib.load_manifest("stereo_click")
import rospy
import math
import tf
from tf.msg import tfMessage
import cv
from std_msgs.msg import String
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import thread
from stereo_click.msg import *

## Basically just outputs a nice image from the camera stream ##
class CameraNode:

	##	The constructor
	#	@param self The object pointer
	#	@param cameraName The name of the camera in the stereo pair. Ex: /wide_stereo_left
	#	@param outputName The name of the output node
	def __init__(self,cameraName,outputName):
		self.name = "%s Viewer"%cameraName
		self.cp = False
		self.ch_x = 0
		self.ch_y = 0
		self.zoom = 1
		self.offset = (0.0,0.0)
		self.outputName = outputName
		self.bridge = CvBridge()
		self.create_window()
		self.cameraTopic = "%s/image_rect_color"%cameraName
		self.cameraInfoTopic = "%s/camera_info"%cameraName
		self.camera_sub = rospy.Subscriber(self.cameraTopic,Image,self.update_background)
		self.camera_info_sub = rospy.Subscriber(self.cameraInfoTopic,CameraInfo,self.update_camera_info)
		
		self.clear_serv = rospy.Service("%s/received"%self.outputName,EmptySrv,self.clear_request)
		self.set_background(cv.CreateImage((500,500),8,3))
	
	##	Creates a window and updates it for the first time
	def create_window(self):
		cv.NamedWindow(self.name)
		cv.WaitKey(25)
		print "Window created"	
		
	##	Sets the background (used for updating the camera stream)
	#	@param background A pointer to the cvImage which will be the background of the window
	def set_background(self,background):
		self.background = background
	
	##	Updates the background, given a new packet of camera data
	#	@param data The camera data (in Image format)
	def update_background(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			cv_image = cv.fromarray(cv_image)
		except CvBridgeError, e:
			print e
		self.set_background(cv_image)
	
	def update_camera_info(self,data):
		self.set_camera_info(data)
		
	def set_camera_info(self,info):
		self.camera_info = info

	##	The listener, which updates the camera feed
	def listen(self):
		
		if isinstance(self.background, np.ndarray):		
			bgimg = cv.CreateImage(self.background.shape[:2],8,3)
			img = cv.CreateImage(self.background.shape[:2],8,3)
			theWidth = self.background.shape[1]
			theHeight = self.background[0]
		else:

			bgimg = cv.CreateImage((self.background.width,self.background.height),8,3)
			img = cv.CreateImage((self.background.width,self.background.height),8,3)
			theWidth = self.background.width
			theHeight = self.background.height

		cv.Copy(self.background,bgimg)
		smallimg = cv.CreateImage((theWidth/self.zoom,theHeight/self.zoom),8,3)
		cv.GetRectSubPix(bgimg,smallimg,(theWidth/(2*self.zoom)+self.offset[0],theHeight/(2*self.zoom)+self.offset[1]))
		cv.Resize(smallimg,img)
		if(self.cp != False):
			cv.Circle(img,self.zoomPt(self.cp.x,self.cp.y),3,cv.RGB(0,255,0),-1)

		cv.Line(img,(self.ch_x-25,self.ch_y),(self.ch_x+25,self.ch_y),cv.RGB(255,255,0))
		cv.Line(img,(self.ch_x,self.ch_y-25),(self.ch_x,self.ch_y+25),cv.RGB(255,255,0))
		cv.ShowImage(self.name,img)
		cv.WaitKey(25)
		
	## Clears the current click point
	def clear_request(self,args):
		self.cp = False
		return []
		
def usage():
	print "clickwindow.py [name] [cameraName] [outputName]"

## Instantiate a new click_window node
def main(args):
#	if len(args) != 3:
#		return usage()
#	[name, cameraName, outputName] = args
	name = "CameraNodeName"
	rospy.init_node(name)
	cameraName = rospy.get_param("~cam","defaultCameraNodeCamera")
	outputName = rospy.get_param("~output","defaultCameraNodeOutput")
	gui = CameraNode(cameraName=cameraName,outputName=outputName)
	while not rospy.is_shutdown():
		gui.listen()
	cv.DestroyAllWindows()

if __name__ == '__main__':
	args = sys.argv[1:]
	try:
		main(args)
	except rospy.ROSInterruptException: pass
