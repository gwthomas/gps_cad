import numpy as np
import actionlib
import rospy
import moveit_commander
import moveit_msgs.msg

rbt = moveit_msgs.msg.RobotTrajectory()
with open('thePlan', 'r') as f:
	data = f.read()


new_rbt = moveit_msgs.msg.RobotTrajectory(data)
