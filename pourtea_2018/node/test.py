#!/usr/bin/env python
#for 2018shanghai, pouring tea robot
#edited by jiali zhang, 20180508
#move arm for test

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

COMMAND = Command_msgs()

class pour_tea:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('pour_tea')
                
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
        rospy.sleep(1)
	
	# Open the effector
	angle1 = UInt16(0)
        self.angle.publish(angle1)
	rospy.sleep(1)
	print "open"

        # Set the target pose
        pose_target = geometry_msgs.msg.Pose()
	pose_target.position.x = 0.340634027263
	pose_target.position.y = 0.033061201953
	pose_target.position.z = 0.414550480234

        pose_target.orientation.x=-2.63478459783e-05
        pose_target.orientation.y=-2.21490562558e-05
        pose_target.orientation.z=2.2024610446e-05
        pose_target.orientation.w=0.999999999165
        
        # Set the start state to the current state
        my_arm.set_start_state_to_current_state()
        
        # Set the goal pose
        my_arm.set_pose_target(pose_target)
        my_arm.go(wait=True)

        # Plan the trajectory to the goal
        traj = my_arm.plan()
        
        # Execute the planned trajectory
        my_arm.execute(traj)
    
        # Pause for a second
        rospy.sleep(1)

	# Close the effector
	angle2 = UInt16(180)
        self.angle.publish(angle2)
	rospy.sleep(1)
	print "close"

        # Move to the named pose "my_arm_sub"
	my_arm.set_named_target('my_arm_sub')
        my_arm.go()
        rospy.sleep(1)

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    pour_tea()

    
    
