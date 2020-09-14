'''
	ENMT482
	Robotics
	Assignment 2
	Coffee Machine Switch
	Daniel Page & Tom Coulson
'''
from numpy import matrix, block, pi, sin, cos, arccos, arctan, dot, linalg, zeros, concatenate 

coffmch_P_switch = matrix([50.67, 35.25, -94.89]).T


theta = -pi/2 # Angle in radians



# Rotation transform
base_R_coffmch = matrix([[cos(theta), 0, sin(theta)],
						 [0,  1, 0],
						 [   -sin(theta),           0,   cos(theta)]])

# Homogenous transform
coffmch_T_switch = block([[base_R_coffmch, coffmch_P_switch], 
						[   zeros(3),          1       ]])

print('coffmch_T_switch = np.{}\n'.format(repr(coffmch_T_switch)))




