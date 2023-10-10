#!/usr/bin/env python3
import numpy as np

class kinematics():
	def __init__(self, dh_parameters, a_n):
		self.alpha = dh_parameters[0]
		self.a = dh_parameters[1]
		self.d = dh_parameters[2]
		self.initial_theta = np.array([0.0, -np.arctan(0.126/0.024), np.arctan(0.126/0.024), 0.0])
		self.a_n = np.array(a_n)
		
		# Frame 1 is shift by 0.012 in x-axis wrt Frame 0
		self.x_shift = 0.012
		
	def forward_kinematics(self, thetas):
		joint_angles = np.array(thetas) + self.initial_theta
		#correction, frame 1 is shifted by 0.012 in x-axis wrt frame 0
		T = self.Homo_transformation(0, 0, self.x_shift, 0)
		for i in range(len(joint_angles)):
			T = np.matmul(T,self.Homo_transformation(joint_angles[i], self.alpha[i], self.a[i], self.d[i]))
		end_pos = np.round(np.matmul(T, self.a_n), 4)[:3] #array format [x, y, z] of end-effector
		return end_pos
		
	def Homo_transformation(self,theta, alpha, a, d):
		T = np.array([[np.cos(theta), -np.sin(theta), 0, a],
		 	[np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -d*np.sin(alpha)],
		 	[np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), d*np.cos(alpha)],
		 	[0, 0, 0, 1]])
		return T
		
	def inverse_kinematics(self, position, phi):
		x = position[0] - self.x_shift
		y = position[1]
		z = position[2]
		
		d1 = self.d[0]
		alpha2 = np.arctan(0.024/0.128)
		a1 = self.a[2]   #0.130
		a2 = self.a[3]   #0.124
		a3 = self.a_n[0] #0.126
		
		r = np.sqrt(x**2 + y**2)
		z_new = z - d1
		
		sm_th = (phi + np.pi) % (2 * np.pi) - np.pi
		if -np.pi/2<=sm_th and sm_th<=np.pi/2:
			x_ph = [[r, phi],[-r, np.pi-phi]]
		else:
			x_ph = [[r, np.pi-phi],[-r, phi]]
		
		theta_1 = np.arctan2(y,x)
		
		theta_2, theta_3, theta_4 = self.P3R_inverse_kinematics(a1, a2, a3, x_ph[0][0], z_new, x_ph[0][1])
		
		theta_2 = np.round(np.pi/2 - theta_2 - alpha2, 3)
		theta_3 = np.round(-np.pi/2 - theta_3 + alpha2, 3)
		theta_4 = np.round(-theta_4, 3)
		
		return np.array([theta_1, theta_2, theta_3, theta_4])
		
	def P3R_inverse_kinematics(self, a1, a2, a3, x, y, theta):
		# Inverse Kinematics of planar 3R robot
		x2 = x - a3*np.cos(theta)
		y2 = y - a3*np.sin(theta)
		
		#D = (x2**2 + y2**2 - (a1**2 + a2**2))/(2*a1*a2)
		#th2 = np.arctan2(-np.sqrt(1 + D**2),D)
		#th1 = np.arctan2(x2, y2) - np.arctan2(a2*np.sin(th2),(a1 + a2*np.cos(th2)))
		#th3 = theta - th1 -th2
		
		cos_th2 = (x2**2+y2**2-a1**2-a2**2)/(2*a1*a2)
		sin_th2 = [-np.sqrt(1-cos_th2**2), np.sqrt(1-cos_th2**2)]
		
		th2 = [np.arctan2(sin_th2[0], cos_th2),np.arctan2(sin_th2[1], cos_th2)]
		
		sin_th1 = [(y2*(a1+a2*np.cos(th2[0]))-a2*np.sin(th2[0])*x2)/(a1**2+a2**2+2*a1*a2*np.cos(th2[0])),(y2*(a1+a2*np.cos(th2[1]))-a2*np.sin(th2[1])*x2)/(a1**2+a2**2+2*a1*a2*np.cos(th2[1]))]
		cos_th1 = [(x2*(a1+a2*np.cos(th2[0]))+a2*np.sin(th2[0])*y2)/(a1**2+a2**2+2*a1*a2*np.cos(th2[0])), (x2*(a1+a2*np.cos(th2[1]))+a2*np.sin(th2[1])*y2)/(a1**2+a2**2+2*a1*a2*np.cos(th2[1]))]
		
		th1 = [np.arctan2(sin_th1[0], cos_th1[0]),np.arctan2(sin_th1[1], cos_th1[1])]
		th3 = [theta-th1[0]-th2[0], theta-th1[1]-th2[1]]
		
		return th1[0], th2[0], th3[0]
		
		
if __name__ == "__main__":
	dh_parameters = np.array([[0, -np.pi/2, 0, 0], [0, 0, 0.130, 0.124], [0.077, 0, 0, 0], 0.0])
	a_n = [0.126, 0, 0, 1]
	
	k = kinematics(dh_parameters, a_n)
	fk = k.forward_kinematics([0.0, 0.0, -0.002, 0.002])
	print("Forward Kinematics: ",list(fk))
	
	ik = k.inverse_kinematics([0.286, 0.0, 0.205], 0)
	print("Inverse Kinematics: ",list(ik))

