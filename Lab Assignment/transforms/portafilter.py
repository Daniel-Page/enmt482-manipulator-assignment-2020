'''
	ENMT482
	Robotics
	Assignment 2
	Coffee Machine Switch
	Daniel Page & Tom Coulson
'''
from numpy import matrix, block, pi, sin, cos, arccos, arctan, dot, linalg, zeros, concatenate 

coffmch_P_switch = matrix([-399.59, -55.10, 534.02]).T


theta = pi # Angle in radians



# Rotation transform
base_R_coffmch = matrix([[cos(theta), 0, sin(theta)],
						 [0,  1, 0],
						 [   -sin(theta),           0,   cos(theta)]])

# Homogenous transform
coffmch_T_switch = block([[base_R_coffmch, coffmch_P_switch], 
						[   zeros(3),          1       ]])

print('coffmch_T_switch = np.{}\n'.format(repr(coffmch_T_switch)))

