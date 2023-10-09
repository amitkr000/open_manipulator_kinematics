#!/usr/bin/env python3
import rospy
from math import cos, sin, pi
import numpy as np
from std_msgs.msg import Float64MultiArray
from open_manipulator_msgs.msg import KinematicsPose
from geometry_msgs.msg import Pose

		 
class Forward_kinematics():
	def __init__(self):
		self.pub = rospy.Publisher('/open_manipular/gripper_pose', KinematicsPose, queue_size=10)
		#self.joint_angles = [joint_angles[0], joint_angles[1] - 90, joint_angles[2]+90, joint_angles[3]] 
		self.joint_angles = []
		#Through modified DH parameter,
		self.alpha = [0, -pi/2, 0, 0]
		self.a = [0, 0, 0.130, 0.124]
		self.d = [0.077, 0, 0, 0]
		self.initial_theta = np.array([0.0, -np.deg2rad(79), np.deg2rad(79), 0.0])
		#End effector position vector respect to 4 frame.
		self.a_n = np.array([0.126, 0, 0, 1])
		
		#Joint angles subscriber
		self.angle_sub = rospy.Subscriber("open_manipulator/joint_angles", Float64MultiArray, self.callback)

		
	def callback(self,msg):
		self.joint_angles = np.array([msg.data[0], msg.data[1], msg.data[2], msg.data[3]])
		#Adding initial theta condition
		self.joint_angles += self.initial_theta
		self.end_effector_pos = KinematicsPose()
		end_pos = self.End_effector()
		self.end_effector_pos.pose.position.x = end_pos[0] + 0.012 #frame 0 and 1 is 0.012 amount shift in x-axis.
		self.end_effector_pos.pose.position.y = end_pos[1]
		self.end_effector_pos.pose.position.z = end_pos[2]
		self.pub.publish(self.end_effector_pos)
		
	def Homo_transformation(self,theta, alpha, a, d):
		T = np.array([[cos(theta), -sin(theta), 0, a],
		 	[sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha)],
		 	[sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha)],
		 	[0, 0, 0, 1]])
		return T
		 	
	def End_effector(self):
		T = self.Homo_transformation(self.joint_angles[0], self.alpha[0], self.a[0], self.d[0])
		#print(T)
		for i in range(1,len(self.joint_angles)):
			T = np.matmul(T,self.Homo_transformation(self.joint_angles[i], self.alpha[i], self.a[i], self.d[i]))

		return np.matmul(T, self.a_n)
		
		 	
if __name__ == "__main__":
	rospy.init_node("/open_manipulator/forward_kinematic")
	Forward_kinematics()
	rospy.spin()


