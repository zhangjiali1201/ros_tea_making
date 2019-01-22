#!/usr/bin/env python
#rewrite for 2018shanghai, pouring tea robot
#edited by jiali zhang, 20180512
#catch the tea, and pour

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

class pour_pose:
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
	print "i am ready "

	# Open the effector
	angle1 = UInt16(0)
        self.angle.publish(angle1)
	rospy.sleep(1)
	#print "open"

        # Set the target pose
        pose_target = geometry_msgs.msg.Pose()
	pose_target.position.x = 0.564893915079
	pose_target.position.y = -0.0396171559636
	pose_target.position.z = 0.127624905831

        pose_target.orientation.x=-8.8568408606e-05
        pose_target.orientation.y=-0.000126488184872
        pose_target.orientation.z=0.000114364423579
        pose_target.orientation.w=0.999999981539

	# Set the goal pose
        my_arm.set_pose_target(pose_target)
        my_arm.go(wait=True)
	rospy.sleep(2)
	print "i have arrived"

	# Close the effector
	angle2 = UInt16(180)
        self.angle.publish(angle2)
	rospy.sleep(1)

	# Move to the named pose "my_arm_sub"
	my_arm.set_named_target('my_arm_sub')
        my_arm.go()
        rospy.sleep(2)

	# Set the pour pose
        pose_target = geometry_msgs.msg.Pose()
	pose_target.position.x = 0.3915
	pose_target.position.y = -8.02980162362e-11
	pose_target.position.z = 0.447

        pose_target.orientation.x=-0.950129222006
        pose_target.orientation.y=9.74425371596e-11
        pose_target.orientation.z=7.05703190035e-11
        pose_target.orientation.w=0.311856475787

	# Set the pour pose
        my_arm.set_pose_target(pose_target)
        my_arm.go(wait=True)
	rospy.sleep(2)

        # Plan the trajectory to the goal
        traj = my_arm.plan()
        
        # Execute the planned trajectory
        my_arm.execute(traj)
    
        # Pause for a second
        rospy.sleep(1)

        # Move to the named pose "my_arm_sub"
	my_arm.set_named_target('my_arm_sub')
        my_arm.go()
        rospy.sleep(2)
	print "i am back"

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    pour_pose()

    
    
