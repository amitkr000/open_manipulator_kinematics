#!/usr/bin/env python3
import numpy as np

class kinematics():
	def __init__(self, dh_parameters, a_n):
		self.alpha = dh_parameters[0]
		self.a = dh_parameters[1]
		self.d = dh_parameters[2]
		self.initial_theta = dh_parameters[3]
		self.a_n = np.array(a_n)
		
	def forward_kinematics(self, thetas):
		joint_angles = np.array(thetas) + self.initial_theta
		#correction, frame 1 is shifted by 0.012 in x-axis wrt frame 0
		T = self.Homo_transformation(0, 0, 0.012, 0)
		for i in range(len(joint_angles)):
			T = np.matmul(T,self.Homo_transformation(joint_angles[i], self.alpha[i], self.a[i], self.d[i]))
		return np.round(np.matmul(T, self.a_n), 3)[:3]
		
	def Homo_transformation(self,theta, alpha, a, d):
		T = np.array([[np.cos(theta), -np.sin(theta), 0, a],
		 	[np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -d*np.sin(alpha)],
		 	[np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), d*np.cos(alpha)],
		 	[0, 0, 0, 1]])
		return T
		
if __name__ == "__main__":
	dh_parameters = np.array([[0, -np.pi/2, 0, 0], [0, 0, 0.130, 0.124], [0.077, 0, 0, 0], [0.0, -np.deg2rad(79), np.deg2rad(79), 0.0]])
	a_n = [0.126, 0, 0, 1]
	
	k = kinematics(dh_parameters, a_n)
	fk = k.forward_kinematics([0.0, 0.0, 0.0, 0.0])
	print(fk)

