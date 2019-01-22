#!/usr/bin/env python
#for 2018shanghai, pouring tea robot
#edited by jiali zhang, 20180502
#add voice to catch_re

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

from sound_play.libsoundplay import SoundClient

COMMAND = Command_msgs()

class pour_tea:
    def __init__(self):
	# Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('pour_tea')

	#set the default TTSvoice to use
        self.voice = rospy.get_param("~voice", "voice_cmu_us_clb_arctic_clunits")

        #Set the wav file path if used
        #self.wavepath = rospy.get_param("~wavepath", script_path + "/../sounds")

        #Create the sound client object
        self.soundhandle = SoundClient()

        #Wait a moment to let the client connect to the sound_play server
        rospy.sleep(1)        
     
        # Initialize the move group for my_arm
	my_arm = moveit_commander.MoveGroupCommander('my_arm')
      
 	# How fast will we check the odometry values?
        rate = 20

	# Set the equivalent ROS rate variable
        r = rospy.Rate(rate)

	# Publisher to control of duoji
        self.angle = rospy.Publisher('/servo', UInt16, queue_size=5)

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

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    pour_tea()

    
    
