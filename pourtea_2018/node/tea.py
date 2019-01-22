#!/usr/bin/env python
#for 2018shanghai, pouring tea robot
#edited by jiali zhang, 20180508
#whole project v1.1

import rospy
import tf
import tf2_ros
import rospy, sys
from moveit_python import *
from moveit_msgs.msg import Grasp, PlaceLocation
import geometry_msgs
from sensor_msgs.msg import JointState
import std_msgs
from std_msgs.msg import UInt16
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from msg_gateway.msg import Command_msgs

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from std_msgs.msg import String,Int16
import actionlib
import actionlib_msgs.msg
from std_srvs.srv import Empty

from sound_play.libsoundplay import SoundClient

from collections import  deque
import numpy as np
#import imutils
import cv2
import time
import freenect

COMMAND = Command_msgs()

class pour_tea:
    def __init__(self, script_path):
	rospy.loginfo("In the __init__()")
        rospy.init_node('pour_tea')
        rospy.on_shutdown(self.cleanup)
        rospy.Subscriber('/voice_cmd', String, self.callback)
	self.door=[-2.696704,-1.474741,0]
	self.rot=[0,0,0.996702,-0.081146]
        self.voice = rospy.get_param("~voice", "voice_don_diphone")
        self.wavepath = rospy.get_param("~wavepath", script_path + "/../sounds")
        self.soundhandle = SoundClient()
        self.exit_point = PoseStamped()
    	self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

    	connected_befor_timeout = self.move_base_client.wait_for_server(rospy.Duration(2.0))
    	if connected_befor_timeout:
    		rospy.loginfo('succeeded connecting to move_base server')
    	else:
        	rospy.loginfo('failed connecting to move_base server')
        	return

	rospy.wait_for_service('move_base/clear_costmaps',5.0)
	self.service_clear_costmap_client = rospy.ServiceProxy('move_base/clear_costmaps',Empty)
	
	rospy.loginfo('connected to move_base/clear_costmaps')
	rospy.loginfo('###### ready ######')
	
        rospy.sleep(1)
        
    # voice
    def sl(self,content,before,after):
	rospy.sleep(before)
	rospy.loginfo(content)
        self.soundhandle.say(content, self.voice)
	rospy.sleep(after)
	
    # pour tea
    def callback(self, msg):
	if(msg.data=="tea"):
		self.sl("i will pour tea for you",2,4)
######### arm ############################
		# Initialize the move_group API
		moveit_commander.roscpp_initialize(sys.argv)

		#set the default TTSvoice to use
		#self.voice = rospy.get_param("~voice", "voice_don_diphone")

		#Create the sound client object
		self.soundhandle = SoundClient()
	     
		# Initialize the move group for my_arm
		my_arm = moveit_commander.MoveGroupCommander('my_arm')
	      
	 	# How fast will we check the odometry values?
		rate = 20

		# Set the equivalent ROS rate variable
		r = rospy.Rate(rate)

		# Publisher to control of duoji
		self.angle = rospy.Publisher('/servo', UInt16, queue_size=5)

		rospy.loginfo('##### arm start #####')

		# Start the arm in the "my_arm_sub" pose stored in the SRDF file
		my_arm.set_named_target('my_arm_sub')
		my_arm.go()
		print "i am ready "
		self.soundhandle.say("i am ready", self.voice)
		rospy.sleep(1)

		# Open the effector
		angle1 = UInt16(0)
		self.angle.publish(angle1)
		rospy.sleep(1)

		# Set the target pose
		pose_target = geometry_msgs.msg.Pose()
		pose_target.position.x = 0.463741790225
		pose_target.position.y = 0.0355905409014
		pose_target.position.z = 0.159723615625

		pose_target.orientation.x=-0.155696563454
		pose_target.orientation.y=-0.0967517483771
		pose_target.orientation.z=-0.0152919219845
		pose_target.orientation.w=0.982936333867
		
		# Set the start state to the current state
		my_arm.set_start_state_to_current_state()
		
		# Set the goal pose
		my_arm.set_pose_target(pose_target)
		my_arm.go(wait=True)
		print "i have arrived"
		self.soundhandle.say("i have arrived", self.voice)
		rospy.sleep(1)

		# Plan the trajectory to the goal
		traj = my_arm.plan()
		
		# Execute the planned trajectory
		my_arm.execute(traj)
	    
		# Pause for a second
		rospy.sleep(1)

		# Close the effector
		angle2 = UInt16(180)
		self.angle.publish(angle2)
		print "i have got it"
		self.soundhandle.say("i have got it", self.voice)
		rospy.sleep(1)


		# Set the target pose
		pose_target = geometry_msgs.msg.Pose()
		pose_target.position.x = 0.00748029077039
		pose_target.position.y = 0.323374542327
		pose_target.position.z = 0.115063324259

		pose_target.orientation.x=-0.0101761048337
		pose_target.orientation.y=-0.136667508585
		pose_target.orientation.z=0.74522355714
		pose_target.orientation.w=0.652579718403

		# Set the goal pose
		my_arm.set_pose_target(pose_target)
		my_arm.go(wait=True)
		rospy.sleep(2)

		# Open the effector
		angle3 = UInt16(0)
		self.angle.publish(angle3)
		print "i will give it to you"
		self.soundhandle.say("i will give it to you", self.voice)
		rospy.sleep(1)

	
		# Set the target pose
		pose_target = geometry_msgs.msg.Pose()
		pose_target.position.x = 0.0440077253782
		pose_target.position.y = 0.18084156521
		pose_target.position.z = 0.413695026221

		pose_target.orientation.x=-3.75311341233e-05
		pose_target.orientation.y=-6.57310621033e-06
		pose_target.orientation.z=0.602072433994
		pose_target.orientation.w=0.7984414711

		# Set the goal pose
		my_arm.set_pose_target(pose_target)
		my_arm.go(wait=True)
		rospy.sleep(1)

		# Close the effector
		angle4 = UInt16(180)
		self.angle.publish(angle4)
		rospy.sleep(1)

		# Move to the named pose "my_arm_sub"
		my_arm.set_named_target('my_arm_sub')
		my_arm.go()
		print "i am back"
		self.soundhandle.say("i am back", self.voice)
		rospy.sleep(2)

		rospy.loginfo('##### arm stop #####')

		# Shut down MoveIt cleanly
		moveit_commander.roscpp_shutdown()
		
		# Exit MoveIt
		moveit_commander.os._exit(0)

######### arm ###################
		return
	
	# go to the side of table
	if(msg.data=="start"):
	    rospy.loginfo("i will go there")
	    goal3 = MoveBaseGoal()
            goal3.target_pose.header.frame_id = 'map'
    	    goal3.target_pose.header.stamp = rospy.Time.now()
    	    goal3.target_pose.pose.position.x = self.door[0]
    	    goal3.target_pose.pose.position.y = self.door[1]
    	    goal3.target_pose.pose.position.z = self.door[2]
	    goal3.target_pose.pose.orientation.x = self.rot[0]
	    goal3.target_pose.pose.orientation.y = self.rot[1]
	    goal3.target_pose.pose.orientation.z = self.rot[2]
	    goal3.target_pose.pose.orientation.w = self.rot[3]
	    self.move_base_client.send_goal(goal3,done_cb=self.door_cb)
	    
	# exam subscribe /voice_cmd
	if(msg.data=="ready"):
		rospy.loginfo("i am ready")

    # find object 
    def door_cb(self,state,result):
	self.sl("i have arrived",2,4)
	self.find_object_with_kinect(np.array([143,36,52]),np.array([200,255,255]),"Cup")

    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down talkback node...")

    def find_object_with_kinect(self,colorLower, colorUpper, Object_Name):
        mybuffer = 64
        pts = deque(maxlen=mybuffer)
    	frame,_ = freenect.sync_get_video()
    	frame=cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
    	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    	mask = cv2.inRange(hsv, colorLower, colorUpper)
    	mask = cv2.erode(mask, None, iterations=2)
    	mask = cv2.dilate(mask, None, iterations=2)
    	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    	center = None
   	if len(cnts) > 0:
	    c = max(cnts, key = cv2.contourArea)
	    ((x, y), radius) = cv2.minEnclosingCircle(c)
	    M = cv2.moments(c)
	    center = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
	    if radius > 10:
		 rect=cv2.minAreaRect(c)
		 cv2.rectangle(frame,(int(x)-int(radius),int(y)-int(radius)),(int(x+radius),int(y+2*radius)),(0,255,0),1)
		 font = cv2.FONT_HERSHEY_SIMPLEX  
	  	 cv2.putText(frame,Object_Name, (int(x)-int(radius),int(y)-int(radius)), font, 1, (255,255,255), 2)  
	  	 pts.appendleft(center)
   	cv2.imwrite("cup.jpg",frame)
	rospy.loginfo("photo saved")

if __name__=="__main__":
    try:
        pour_tea(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("pour_tea node terminated.")
