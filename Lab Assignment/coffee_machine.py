'''
	ENMT482
	Robotics
	Assignment 2
	Grinder Functions
	Daniel Page & Tom Coulson
'''


import robodk as rdk # Robot toolbox
import numpy as np


def base_T_coffmch():
	# Coffee machine origin in the base reference frame
	base_P_coffmch = np.matrix([-359.9, -387.38, 341.24]).T

	# Coffee machine additional point
	base_P_adpnt = np.matrix([-573.54, -443.66, 341.24]).T

	# The change in coordinates with respect to the base
	dspmt_vector = base_P_adpnt - base_P_coffmch

	# Determine the angle from the y axis of the displacement vector
	dspmt_xy_vector = [float(dspmt_vector[0]), float(dspmt_vector[1])] # The displacement in terms of x and y

	# Determine the change in angle between the previous vector point to the new additional point
	coffmch_P_match = np.matrix([0, 218, 0]).T
	coffmch_P_match_xy = [float(coffmch_P_match[0]), float(coffmch_P_match[1])]

	theta_base = np.arctan2(dspmt_xy_vector[1], dspmt_xy_vector[0])
	theta_coffmch = np.arctan2(coffmch_P_match_xy[1], coffmch_P_match_xy[0])
	theta = theta_base - theta_coffmch

	# Coffee machine reference frame with respect to the robot base reference frame
	return rdk.TxyzRxyz_2_Pose([base_P_coffmch[0], base_P_coffmch[1], base_P_coffmch[2], 0, 0, theta])


def rotate_arm_T():
	# Rotate the end effector 50 degees
	return rdk.TxyzRxyz_2_Pose([0, 0, 0,0,0, np.radians(50)])


def tamper_to_coffee_machine(robot, delay):
	# Move to a position where the portafilter can be manually removed and connect to the coffee machine
	
	# Intermediate positions
	servo_positions = [[-16.07, -68.85, -127.0, 204.73-360, -122.14, 144.74],
	[-61.07, -68.85, -127.0, 204.73-360, -122.14, 144.74],
	[-109.29, -68.85, -127.0, 204.73-360, -122.14, 144.74]]

	
	for pos in servo_positions:
		robot.MoveJ(pos)

	rdk.pause(delay)


def place_cup_in_coffmch(robot):
	# Rotate the cup and move to the alcove in the coffee machine
	
	# Intermediate positions
	servo_positions = [[-82.89, -77.5, -129.83, -152.65, 293.39, -39.99],
	[-82.89, -77.5, -129.83, -152.65, 293.39, 140.0],
	[-67.63446698231493, -62.20658978885603, -138.84207898275827, -158.08374327970063, 357.71526342867105, 139.1987039433821]]

	for pos in servo_positions:
		robot.MoveJ(pos)

	# Frames
	x_offset = 8
	coffmch_T_cup_stand = rdk.TxyzRxyz_2_Pose([-12.68+x_offset, 72.0, -290.0, -np.pi/2, 0, -np.pi/2])
	cup_tool_center_T_tcp = rdk.TxyzRxyz_2_Pose([-47.0, 0, 186.11, 0, 0, 0]).invH()
	top_of_cup = rdk.TxyzRxyz_2_Pose([85, 0, 0, np.radians(20), 0, 0])

	coffmch_T_base = base_T_coffmch()*coffmch_T_cup_stand*top_of_cup*cup_tool_center_T_tcp*rotate_arm_T()

	robot.MoveJ(coffmch_T_base)


def exit_cup_standoff(robot):
	# Exit out of the stand-off of the coffee machine

	#servo_positions = [[-67.63446698231493, -62.20658978885603, -138.84207898275827, -158.08374327970063, 357.71526342867105, 139.1987039433821]]

	#for pos in servo_positions:
		#robot.MoveJ(pos)

	# Frames
	x_offset = 8
	coffmch_T_cup_stand = rdk.TxyzRxyz_2_Pose([-12.68+x_offset, 72.0-140, -290.0, -np.pi/2, 0, -np.pi/2])
	cup_tool_center_T_tcp = rdk.TxyzRxyz_2_Pose([-47.0, 0, 186.11, 0, 0, 0]).invH()
	top_of_cup = rdk.TxyzRxyz_2_Pose([85, 0, 0, np.radians(20), 0, 0])

	coffmch_T_base = base_T_coffmch()*coffmch_T_cup_stand*top_of_cup*cup_tool_center_T_tcp*rotate_arm_T()

	robot.MoveJ(coffmch_T_base)


def stand_off_to_tool_stand(robot):
	# Exit out of the stand-off of the coffee machine

	servo_positions = [[-65.82629367763043, -62.17994892080185, -95.06136286995276, -202.75721955391546, 349.4155545987202, 139.99882594375595],
	[-82.89, -62.17, -95.06, -110.5, 86.73, 139.99],
	[-86.28828808204076, -54.1538870839103, -98.06410880758567, -115.32225213292881, 86.87022399534786, 136.58962302063475],
	[-139.74, -54.15, -98.06, -115.32, 86.87, 136.58],
	[-177.40054272309135, -72.96472522786271, -82.70192714893811, -114.68829201006866, 91.6144420755773, -112.4722784717101]]


	for pos in servo_positions:
		robot.MoveJ(pos)

	# Approach the cup tool on the stand
	T_cup_tool_approach = rdk.TxyzRxyz_2_Pose([-358.13, 102.68, 532.67, 0, np.pi, np.radians(340)])
	
	robot.MoveL(T_cup_tool_approach)


def approach_grinder_tool_cup(robot):
	# Approach the tool stand entry point for the grinder tool from the cup tool
	
	base_T_grinder_tool_approach = rdk.TxyzRxyz_2_Pose([-441.77, -214.48, 534.24, 0, np.pi, np.radians(250)])
	robot.MoveJ(base_T_grinder_tool_approach)


def coffee_switch(robot, delay):
	# Turn the coffee switch on and off with a delay between

	# Frames
	pointer_T_end_tcp = rdk.TxyzRxyz_2_Pose([0, 0, 102.82, 0, 0, 0]).invH()
	coffmch_T_coffee_switch = rdk.TxyzRxyz_2_Pose([50.67, 35.25, -27.89, 0, np.radians(-90), 0])
	coffee_switch_T_on_apprch = rdk.TxyzRxyz_2_Pose([-7, 0, -15, 0, 0, 0])
	coffee_switch_T_on = rdk.TxyzRxyz_2_Pose([-7, 0, -2, 0, 0, 0])
	coffee_switch_T_off_apprch = rdk.TxyzRxyz_2_Pose([7, 0, -15, 0, 0, 0])
	coffee_switch_T_off = rdk.TxyzRxyz_2_Pose([7, 0, -2, 0, 0, 0])
	
	base_T_on_apprch = base_T_coffmch()*coffmch_T_coffee_switch*pointer_T_end_tcp*coffee_switch_T_on_apprch*rotate_arm_T()
	base_T_on = base_T_coffmch()*coffmch_T_coffee_switch*pointer_T_end_tcp*coffee_switch_T_on*rotate_arm_T()
	base_T_off_apprch = base_T_coffmch()*coffmch_T_coffee_switch*pointer_T_end_tcp*coffee_switch_T_off_apprch*rotate_arm_T()
	base_T_off = base_T_coffmch()*coffmch_T_coffee_switch*pointer_T_end_tcp*coffee_switch_T_off*rotate_arm_T()

	robot.MoveJ(base_T_on_apprch)
	robot.MoveJ(base_T_on)
	robot.MoveJ(base_T_on_apprch)

	rdk.pause(delay)
	
	robot.MoveJ(base_T_off_apprch)
	robot.MoveJ(base_T_off)
	robot.MoveJ(base_T_off_apprch)


def coffee_switch_to_stand(robot):
	# Move from the coffee switch to the tool stand entry point for the grinder tool

	servo_positions = [[-158.64797450496505, -100.97596345809919, -92.64644344681786, -166.3759085755322, 186.59322255491935, -219.99840363028815],
	[-158.18, -92.52, -102.2, -165.26, 91.49, -219.99],
	[-149.4742563684273, -77.38278147993852, -90.90689728227224, -108.10968801036518, 92.48517960654515, -211.3358198302056]]

	for pos in servo_positions:
		robot.MoveJ(pos)

	base_T_grinder_tool_approach = rdk.TxyzRxyz_2_Pose([-441.77, -214.48, 534.24, 0, np.pi, np.radians(250)])
	robot.MoveJ(base_T_grinder_tool_approach)


def approach_cup_tool_grinder(robot):
	# Approach the tool stand entry point for the cup tool from the grinder tool
	
	T_cup_tool_approach = rdk.TxyzRxyz_2_Pose([-358.13, 102.68, 532.67, 0, np.pi, np.radians(250)])
	robot.MoveL(T_cup_tool_approach)


def approach_stand_off(robot):
	# Approach the coffee machine stand-off
	
	# Intermediate points
	servo_positions = [[-177.40054272309135, -72.96472522786271, -82.70192714893811, -114.68829201006866, 91.6144420755773, -112.4722784717101],
	[-139.74, -54.15, -98.06, -115.32, 86.87, 136.58],
	[-86.28828808204076, -54.1538870839103, -98.06410880758567, -115.32225213292881, 86.87022399534786, 136.58962302063475],
	[-82.89, -62.17, -95.06, -110.5, 86.73, 139.99],
	[-65.82629367763043, -62.17994892080185, -95.06136286995276, -202.75721955391546, 349.4155545987202, 139.99882594375595]]


	for pos in servo_positions:
		robot.MoveJ(pos)

	# Frames
	x_offset = 8
	coffmch_T_cup_stand = rdk.TxyzRxyz_2_Pose([-12.68+x_offset, 72.0-140, -290.0, -np.pi/2, 0, -np.pi/2])
	cup_tool_center_T_tcp = rdk.TxyzRxyz_2_Pose([-47.0, 0, 186.11, 0, 0, 0]).invH()
	top_of_cup = rdk.TxyzRxyz_2_Pose([85, 0, 0, np.radians(20), 0, 0])

	coffmch_T_base = base_T_coffmch()*coffmch_T_cup_stand*top_of_cup*cup_tool_center_T_tcp*rotate_arm_T()

	robot.MoveJ(coffmch_T_base)


def skewer_filled_cup(robot):
	# Skewer the filled cup, lift and remove from the stand-off
	
	# Frames
	x_offset = 8
	coffmch_T_cup_stand = rdk.TxyzRxyz_2_Pose([-12.68+x_offset, 72.0, -290.0, -np.pi/2, 0, -np.pi/2])
	cup_tool_center_T_tcp = rdk.TxyzRxyz_2_Pose([-47.0, 0, 186.11, 0, 0, 0]).invH()
	top_of_cup = rdk.TxyzRxyz_2_Pose([85, 0, 0, np.radians(20), 0, 0])

	coffmch_T_base = base_T_coffmch()*coffmch_T_cup_stand*top_of_cup*cup_tool_center_T_tcp*rotate_arm_T()

	robot.MoveJ(coffmch_T_base)



def serve_cup(robot):
	# Move the cup into a position where it can be taken
	
	servo_positions = [[-59.54399725279543, -58.0611228121563, -132.65521635312516, -169.2800646851474, 355.6978510185116, 139.99668359351074],
	[-59.54399725279543, -59.9347314478943, -102.27954629484765, -197.78212610768693, 355.6978510185116, 139.99668359351074],
	[-2.37, -59.93, -102.27, -197.78, 355.69, 139.99],
	[-3.5245101100421348, -80.96092040863095, -84.56663573942096, -194.4426402132363, 273.28554858957295, 139.9880557989964]]

	for pos in servo_positions:
		robot.MoveJ(pos)
