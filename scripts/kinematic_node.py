#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from open_manipulator_kinematics.msg import KinematicPose
from geometry_msgs.msg import Pose
import omx_kinematics

		 
class kinematics_node():
	def __init__(self):
		#Open_manipulator parameters
		self.dh_parameters = np.array([[0, -np.pi/2, 0, 0], [0, 0, 0.130, 0.124], [0.077, 0, 0, 0]])
		self.a_n = [0.126, 0, 0, 1]
		
		#Subscribing data
		self.joint_angles = None
		self.gripper_kinematic = None
		
		#Publishers
		self.pos_pub = rospy.Publisher('/omx_kinematics/gripper_kinematic', KinematicPose, queue_size=0)
		self.angle_pub = rospy.Publisher('/omx_kinematics/joint_state', Float64MultiArray, queue_size=0)
		
		#Subscribers
		self.angle_sub = rospy.Subscriber("omx_kinematics/joint_angles", Float64MultiArray, self.angle_callback)
		self.pos_sub = rospy.Subscriber("omx_kinematics/gripper_pose", Float64MultiArray, self.pos_callback)
		
		
	def angle_callback(self, msg):
		self.joint_angles = msg.data
		
	def pos_callback(self, msg):
		self.gripper_kinematic = msg.data
		
	def publish(self):
		k = omx_kinematics.kinematics(self.dh_parameters, self.a_n)
		rate = rospy.Rate(60)
		
		#msgs
		kpose_msg = KinematicPose()
		angle_msg = Float64MultiArray()
		
		while not rospy.is_shutdown():
			#Forward Kinematics
			if (self.joint_angles != None):
				end_pose = k.forward_kinematics(self.joint_angles)
				kpose_msg.position = end_pose[:3]
				kpose_msg.orientation = [end_pose[3], 0.0, 0.0]
				self.pos_pub.publish(kpose_msg)
				
			#Inverse Kinematics
			if (self.gripper_kinematic != None):
				angles = k.inverse_kinematics(self.gripper_kinematic[:3], self.gripper_kinematic[3])
				angle_msg.data = angles
				self.angle_pub.publish(angle_msg)
			rate.sleep()
		 	
if __name__ == "__main__":
	rospy.init_node("omx_kinematics")
	rospy.loginfo("omx_kinematics node has started")
	kinematics_node().publish()
	rospy.loginfo("omx_kinematics node has stopped")
