'''
	ENMT482
	Robotics
	Assignment 2
	Coffee Machine Homogenous Transform
	Daniel Page & Tom Coulson
'''

from numpy import matrix, block, pi, sin, cos, arccos, arctan, dot, linalg, zeros, concatenate 

# Coffee machine origin in the base reference frame
base_P_coffmch = matrix([-359.90, -387.38, 341.24]).T

# Coffee machine additional point
base_P_adpnt = matrix([-573.54, -443.66, 341.24]).T

# The change in coordinates with respect to the base
dspmt_vector = base_P_adpnt - base_P_coffmch

# Determine the angle from the y axis of the displacement vector
base_Y_vector = [0, 1] # Unit vector in the positive x direction of the base
dspmt_xy_vector = [float(dspmt_vector[0]), float(dspmt_vector[1])] # The displacement in terms of x and y

base_Y_unit_vector = base_Y_vector / linalg.norm(base_Y_vector)
dspmt_xy_unit_vector = dspmt_xy_vector / linalg.norm(dspmt_xy_vector)

dot_product = dot(base_Y_unit_vector, dspmt_xy_unit_vector)

theta = arccos(dot_product) # Angle in radians

# Rotation transform
base_R_coffmch = matrix([[cos(theta), -sin(theta), 0],
						 [sin(theta),  cos(theta), 0],
						 [    0,           0,      1]])

# Homogenous transform
base_T_coffmch = block([[base_R_coffmch, base_P_coffmch], 
						[   zeros(3),          1       ]])

print('base_T_coffmch =\n{}\n'.format(base_T_coffmch))

coffmch_P_loc = matrix([0.00, 218.00, 0.00]).T

base_P_loc = base_T_coffmch*concatenate((coffmch_P_loc, matrix([1])))
base_P_loc = base_P_loc[0:3]

print('base_P_loc =\n{}\n'.format(base_P_loc))
