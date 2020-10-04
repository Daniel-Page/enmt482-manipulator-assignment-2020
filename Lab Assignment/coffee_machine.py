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
	servo_positions = [[-79.59, -77.5, -129.83, -152.65, 242.39, -40.0],
	[-79.59, -77.5, -129.83, -152.65, 242.39, 140.0],
	[-92.59, -77.5, -129.83, -152.65, 221.39, 140.0],
	[-145.72, -84.29, -124.34, -151.31, 185.24, 140.0],
	[-176.72, -84.29, -124.34, -151.31, 185.24, 140.0],
	[-181.99354512979588, -94.56088662731848, -114.40134404497128, -160.3369890401527, 179.96589583533623, 130.6413665632621],
	[-180.30276091664734, -107.13262122134887, -98.74624893351948, -153.93056255453772, 181.65713740901614, 140.13107224802914],
	[-173.19267247886944, -107.13711269729652, -98.72482164643942, -154.07763542390902, 195.75739904796106, 139.9598552286246]]

	for pos in servo_positions:
		robot.MoveJ(pos)

	# Frames
	coffmch_T_cup_stand = rdk.TxyzRxyz_2_Pose([-12.68, 72.0, -290.0, 0, -np.pi/2 , 0])
	cup_tool_center_T_tcp = rdk.TxyzRxyz_2_Pose([-47.0, 0, 186.11, 0, 0, 0]).invH()
	top_of_cup = rdk.TxyzRxyz_2_Pose([85, 0, 0, np.radians(20), 0, 0])

	coffmch_T_base = base_T_coffmch()*coffmch_T_cup_stand*top_of_cup*cup_tool_center_T_tcp*rotate_arm_T()

	robot.MoveJ(coffmch_T_base)


def exit_cup_standoff(robot):
	# Exit out of the stand-off of the coffee machine

	# Frames
	coffmch_T_cup_stand = rdk.TxyzRxyz_2_Pose([-12.68, 72.0, -290.0, 0, -np.pi/2 , 0])
	cup_tool_center_T_tcp = rdk.TxyzRxyz_2_Pose([-47, 0, 186.11, 0, 0, 0]).invH()
	top_of_cup_lower = rdk.TxyzRxyz_2_Pose([70, 0, 0, 0, 0, 0])
	top_of_cup_exit = rdk.TxyzRxyz_2_Pose([70, 0, -120, 0, 0, 0])
	
	coffmch_T_lower = base_T_coffmch()*coffmch_T_cup_stand*cup_tool_center_T_tcp*top_of_cup_lower*rotate_arm_T()
	coffmch_T_exit = base_T_coffmch()*coffmch_T_cup_stand*cup_tool_center_T_tcp*top_of_cup_exit*rotate_arm_T()
	
	robot.MoveJ(coffmch_T_lower)
	robot.MoveJ(coffmch_T_exit)


def stand_off_to_tool_stand(robot):
	# Exit out of the stand-off of the coffee machine

	servo_positions = [[-167.87416166874038, -112.25975478474284, -101.93381770960812, -145.7871740485695, 195.3560951845067, 139.99868056083272],
	[-164.72997979411196, -92.22299243751384, -108.93222155002483, -158.7745260578508, 219.47040975684635, 139.96004613605433],
	[-191.23535154757727, -69.82530359526434, -103.71808476336204, -186.25754129444843, 192.96509247508752, 140.0998042166975],
	[-194.6381345228634, -59.61235299149398, -112.65027900764453, -85.2538462288647, 193.27038069741465, 155.09023956404948],
	[-194.63, -59.61, -112.65, -85.25, 93.86, 155.09],
	[-177.1147161812992, -68.83786083779691, -94.57049814745048, -101.4506822105205, 91.37818956333713, 172.55537375525967]]

	for pos in servo_positions:
		robot.MoveJ(pos)

	# Approach the cup tool on the stand
	T_cup_tool_approach = rdk.TxyzRxyz_2_Pose([-358.13, 102.68, 532.67, 0, np.pi, 0])
	
	robot.MoveL(T_cup_tool_approach)


def approach_grinder_tool_cup(robot):
	# Approach the tool stand entry point for the grinder tool from the cup tool
	
	base_T_grinder_tool_approach = rdk.TxyzRxyz_2_Pose([-441.77, -214.48, 534.24, 0, np.pi, 0])
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

	base_T_grinder_tool_approach = rdk.TxyzRxyz_2_Pose([-441.77, -214.48, 534.24, 0, np.pi, 0])
	robot.MoveJ(base_T_grinder_tool_approach)


def approach_cup_tool_grinder(robot):
	# Approach the tool stand entry point for the cup tool from the grinder tool
	
	T_cup_tool_approach = rdk.TxyzRxyz_2_Pose([-358.13, 102.68, 532.67, 0, np.pi, 0])
	robot.MoveL(T_cup_tool_approach)


def approach_stand_off(robot):
	# Approach the coffee machine stand-off
	
	# Frames
	coffmch_T_cup_stand = rdk.TxyzRxyz_2_Pose([-12.68, 72.0, -290.0, 0, -np.pi/2 , 0])
	cup_tool_center_T_tcp = rdk.TxyzRxyz_2_Pose([-47, 0, 186.11, 0, 0, 0]).invH()
	top_of_cup_apprch = rdk.TxyzRxyz_2_Pose([85, 0, -80, 0, 0, 0])

	coffmch_T_exit = base_T_coffmch()*coffmch_T_cup_stand*cup_tool_center_T_tcp*top_of_cup_apprch*rotate_arm_T()
	
	robot.MoveJ(coffmch_T_exit)


def skewer_filled_cup(robot):
	# Skewer the filled cup, lift and remove from the stand-off
	
	# Frames
	coffmch_T_cup_stand = rdk.TxyzRxyz_2_Pose([-12.68, 72.0, -290.0, 0, -np.pi/2 , 0])
	cup_tool_center_T_tcp = rdk.TxyzRxyz_2_Pose([-47, 0, 186.11, 0, 0, 0]).invH()
	top_of_cup_apprch = rdk.TxyzRxyz_2_Pose([85, 0, 0, 0, 0, 0])

	coffmch_T_exit = base_T_coffmch()*coffmch_T_cup_stand*cup_tool_center_T_tcp*top_of_cup_apprch*rotate_arm_T()
	
	robot.MoveJ(coffmch_T_exit)


def serve_cup(robot):
	# Move the cup into a position where it can be taken
	
	servo_positions = [[-161.16981624282218, -117.59676553986716, -91.18715501805069, -151.21335524383355, 184.07138081882417, -219.99735968563965],
	[-160.63949039166687, -104.49897214023892, -108.49829194067634, -146.9739030732602, 228.45005592599293, -219.99117962939158],
	[-177.51597074734343, -76.72836797170052, -133.20685935284234, -150.00140993614133, 209.01461155796153, -219.96441613826596],
	[-177.51597074734343, -70.73257566348516, -96.51514752009028, -192.68891407710876, 209.01461155796153, -219.96441613826596],
	[-120.79, -70.73, -96.51, -192.68, 209.01, -219.96],
	[-61.58, -70.73, -96.51, -192.68, 209.01, -219.96],
	[-46.70777557340955, -70.23720437019183, -96.8199209186163, -192.85805649344917, 207.30195381549416, -219.95457591261194],
	[-17.924259585914672, -101.52699820053837, -67.98468103061732, -190.4414406942232, 236.08545309506553, -220.0037885128249]]

	for pos in servo_positions:
		robot.MoveJ(pos)
