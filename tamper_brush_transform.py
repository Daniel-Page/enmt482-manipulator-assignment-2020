'''
	ENMT482
	Robotics
	Assignment 2
	Tamper-Brush Homogenous Transform
	Daniel Page & Tom Coulson
'''

from numpy import matrix, block, pi, sin, cos, arccos, arctan, dot, linalg, zeros, concatenate 

# Tamper-brush origin in the base reference frame
base_P_tamperbr = matrix([598.10, 4.31, 212.58]).T

# Tamper-bush addditional point
base_P_adpnt = matrix([557.50, 73.18, 157.58]).T

# The change in coordinates with respect to the base
dspmt_vector = base_P_adpnt - base_P_tamperbr

# Determine the angle from the x axis of the displacement vector
base_X_vector = [1, 0] # Unit vector in the positive x direction of the base
dspmt_xy_vector = [float(dspmt_vector[0]), float(dspmt_vector[1])] # The displacement in terms of x and y

base_X_unit_vector = base_X_vector / linalg.norm(base_X_vector)
dspmt_xy_unit_vector = dspmt_xy_vector / linalg.norm(dspmt_xy_vector)

dot_product = dot(base_X_unit_vector, dspmt_xy_unit_vector)

base_theta = arccos(dot_product) # Angle in radians

# Determine the change in angle between the previous vector point to the new additional point
tamperbr_P_adpnt = matrix([70.00, 0.00, -32.00]).T
tamperbr_P_match = matrix([-80.00, 0.00, -55.00]).T

tamperbr_P_adpnt_xy = [float(tamperbr_P_adpnt[0]), float(tamperbr_P_adpnt[1])]
tamperbr_P_match_xy = [float(tamperbr_P_match[0]), float(tamperbr_P_match[1])]

tamperbr_P_adpnt_unit_xy = tamperbr_P_adpnt_xy / linalg.norm(tamperbr_P_adpnt_xy)
tamperbr_P_match_unit_xy = tamperbr_P_match_xy / linalg.norm(tamperbr_P_match_xy)

dot_product = dot(tamperbr_P_adpnt_unit_xy, tamperbr_P_match_unit_xy)

tamperbr_theta = arccos(dot_product) # Angle in radians

# Dot product angle is always positive
# The signs of the angles need to be added manually
theta = base_theta - tamperbr_theta

# Rotation transform
base_R_tamperbr = matrix([[cos(theta), -sin(theta), 0],
						  [sin(theta),  cos(theta), 0],
						  [    0,           0,      1]])

# Homogenous transform
base_T_tamperbr = block([[base_R_tamperbr, base_P_tamperbr], 
						[   zeros(3),          1       ]])

print('base_T_grinder =\n{}\n'.format(base_T_tamperbr))

tamperbr_P_loc = matrix([-80.00, 0.00, -55.00]).T

base_P_loc = base_T_tamperbr*concatenate((tamperbr_P_loc, matrix([1])))
base_P_loc = base_P_loc[0:3]

print('base_P_loc =\n{}\n'.format(base_P_loc))
