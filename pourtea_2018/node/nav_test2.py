#!/usr/bin/env python
#for 2018shanghai, pouring tea robot
#edited by jiali zhang, 20180511
#nav test with clear costmap

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

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult,MoveBaseActionResult
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
        self.base_sts = ' '
	rospy.loginfo("In the __init__()")
        rospy.init_node('pour_tea')
        rospy.on_shutdown(self.cleanup)
        rospy.Subscriber('/voice_cmd', String, self.callback)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.base_status_callback)
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
		rospy.loginfo("##### catch #####")
		rospy.loginfo("##### catch #####")
		return
	
	# go to the side of table
	if(msg.data=="start"):
	    rospy.loginfo("i will go to find tea bag")
	    goal3 = MoveBaseGoal()
            goal3.target_pose.header.frame_id = 'map'
    	    goal3.target_pose.header.stamp = rospy.Time.now()
    	    goal3.target_pose.pose.position.x = -2.290960
    	    goal3.target_pose.pose.position.y = -5.662451
    	    goal3.target_pose.pose.position.z = 0
	    goal3.target_pose.pose.orientation.x = 0
	    goal3.target_pose.pose.orientation.y = 0
	    goal3.target_pose.pose.orientation.z = 0.662838
	    goal3.target_pose.pose.orientation.w = -0.748763
	    self.move_base_client.send_goal(goal3,done_cb=self.tea_cb)
            while True:
	        if self.base_sts != 3 and self.base_sts != 4:
                    pass
                elif self.base_sts == 4:
                    add_two_ints=rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
	            resp1 = add_two_ints()
                    self.base_sts = None
                    rospy.sleep(0.5)
                    self.move_base_client.send_goal(goal3,done_cb=self.costmap_cb)
                else:
                    self.move_base_client.send_goal(goal3,done_cb=self.tea_cb)
                    break	    
	    goal2 = MoveBaseGoal()
            goal2.target_pose.header.frame_id = 'map'
    	    goal2.target_pose.header.stamp = rospy.Time.now()
    	    goal2.target_pose.pose.position.x = -8.029032
    	    goal2.target_pose.pose.position.y = -4.406152
    	    goal2.target_pose.pose.position.z = 0
	    goal2.target_pose.pose.orientation.x = 0
	    goal2.target_pose.pose.orientation.y = 0
	    goal2.target_pose.pose.orientation.z = 0.991846
	    goal2.target_pose.pose.orientation.w = -0.127442
	    self.move_base_client.send_goal(goal2,done_cb=self.cup_cb)
	    
	# exam subscribe /voice_cmd
	if(msg.data=="ready"):
		rospy.loginfo("i am ready")

	# exam arm
	if(msg.data=="examation"):
		rospy.loginfo("i'm okey")
		rospy.loginfo("##### test #####")
		rospy.loginfo("##### test #####")

    # find tea bag 
    def tea_cb(self,state,result):
	self.sl("i have arrived tea bag",2,4)
	rospy.loginfo("##### find tea bag #####")
	rospy.loginfo("##### find tea bag #####")

    # find cup 
    def cup_cb(self,state,result):
	self.sl("i have arrived cup",2,4)
	rospy.loginfo("##### find cup #####")
	rospy.loginfo("##### find cup #####")

    def costmsp_cb(self,state,result):
	self.sl("i failed, costmap",2,4)

    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down talkback node...")

    def base_status_callback(self, msg):
    	self.base_sts = msg.status.status

    
if __name__=="__main__":
    try:
        pour_tea(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("pour_tea node terminated.")
