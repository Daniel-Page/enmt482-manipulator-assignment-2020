'''
	ENMT482
	Robotics
	Assignment 2
	Tool Stand Homogenous Transform
	Daniel Page & Tom Coulson
'''

from numpy import degrees, matrix, block, pi, sin, cos, arccos, arctan, dot, linalg, zeros, concatenate 


base_P_toolstd = matrix([-552.66, -77.19, 19.05]).T

# Middle tool
base_P_adpnt = matrix([-399.59, -55.10, 534.02]).T


dspmt_vector = base_P_adpnt - base_P_toolstd
print(dspmt_vector)

# Determine the angle from the y axis of the displacement vector
base_Y_vector = [0, 1] # Unit vector in the positive x direction of the base
dspmt_xy_vector = [float(dspmt_vector[0]), float(dspmt_vector[1])] # The displacement in terms of x and y

base_Y_unit_vector = base_Y_vector / linalg.norm(base_Y_vector)
dspmt_xy_unit_vector = dspmt_xy_vector / linalg.norm(dspmt_xy_vector)

dot_product = dot(base_Y_unit_vector, dspmt_xy_unit_vector)

theta = arccos(dot_product) # Angle in radians
print(degrees(theta))


# Determine the change in angle between the previous vector point to the new additional point
tamperbr_P_adpnt = matrix([70.00, 0.00, -32.00]).T
tamperbr_P_match = matrix([-80.00, 0.00, -55.00]).T

tamperbr_P_adpnt_xy = [float(tamperbr_P_adpnt[0]), float(tamperbr_P_adpnt[1])]
tamperbr_P_match_xy = [float(tamperbr_P_match[0]), float(tamperbr_P_match[1])]

tamperbr_P_adpnt_unit_xy = tamperbr_P_adpnt_xy / linalg.norm(tamperbr_P_adpnt_xy)
tamperbr_P_match_unit_xy = tamperbr_P_match_xy / linalg.norm(tamperbr_P_match_xy)

dot_product = dot(tamperbr_P_adpnt_unit_xy, tamperbr_P_match_unit_xy)

tamperbr_theta = arccos(dot_product) # Angle in radians
