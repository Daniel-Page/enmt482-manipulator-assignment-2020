'''
	ENMT482
	Robotics
	Assignment 2
	Coffee Grinder Homogenous Transform
	Daniel Page & Tom Coulson
'''

from numpy import matrix, block, pi, sin, cos, arccos, arctan, dot, linalg, zeros, concatenate 

# Grinder origin in the base reference frame
base_P_grinder = matrix([484.51, -426.60, 318.38]).T

# Grinder additional point in the base reference frame
base_P_adpnt = matrix([369.74, -320.06, 67.83]).T

# The change in coordinates with respect to the base
dspmt_vector = base_P_adpnt - base_P_grinder

base_X_vector = [1, 0] # Unit vector in the positive x direction of the base
dspmt_xy_vector = [float(dspmt_vector[0]), float(dspmt_vector[1])] # The displacement in terms of x and y

base_X_unit_vector = base_X_vector / linalg.norm(base_X_vector)
dspmt_xy_unit_vector = dspmt_xy_vector / linalg.norm(dspmt_xy_vector)

dot_product = dot(base_X_unit_vector, dspmt_xy_unit_vector)

theta = arccos(dot_product) # Angle in radians

# Rotation transform
base_R_grinder = matrix([[cos(theta), -sin(theta), 0],
						 [sin(theta),  cos(theta), 0],
						 [    0,           0,      1]])

# Homogenous transform
base_T_grinder = block([[base_R_grinder, base_P_grinder], 
						[   zeros(3),          1       ]])

print('base_T_grinder =\n{}\n'.format(base_T_grinder))

grinder_P_loc = matrix([157.61, 0, -250.45]).T

base_P_loc = base_T_grinder*concatenate((grinder_P_loc, matrix([1])))
base_P_loc = base_P_loc[0:3]

print('base_P_loc =\n{}\n'.format(base_P_loc))
