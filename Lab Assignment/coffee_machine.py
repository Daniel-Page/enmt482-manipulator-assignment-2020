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
	# Coffee machine reference frame with respect to the robot base reference frame
	return rdk.Mat([[-2.5474282622E-01, -9.6700883785E-01, 0.0000000000E+00,
         -3.5990000000E+02],
        [9.6700883785E-01, -2.5474282622E-01, 0.0000000000E+00,
         -3.8738000000E+02],
        [0.0000000000E+00, 0.0000000000E+00, 1.0000000000E+00,
         3.4124000000E+02],
        [0.0000000000E+00, 0.0000000000E+00, 0.0000000000E+00,
         1.0000000000E+00]])


def rotate_arm_T():
	# Rotate the end effector 50 degees
	return rdk.TxyzRxyz_2_Pose([0, 0, 0,0,0, np.radians(50)])


def place_cup_in_coffmch(robot):
	# Rotate the cup and move to the alcove in the coffee machine
	
	# Intermediate positions
	servo_positions = [[-79.59, -77.5, -129.83, -152.65, 242.39, -40.0],
	[-79.59, -77.5, -129.83, -152.65, 242.39, 140.0],
	[-92.59, -77.5, -129.83, -152.65, 221.39, 140.0],
	[-145.72, -84.29, -124.34, -151.31, 185.24, 140.0],
	[-176.72, -84.29, -124.34, -151.31, 185.24, 140.0],
	[-176.72, -84.29, -124.34, -151.31, 202.24, 140.0],
	[-163.00884270390333, -91.09198185429123, -117.87640611155409, -150.99315056155402, 215.95100259871427, 139.97571282346712]]

	for pos in servo_positions:
		robot.MoveJ(pos)

	# Frames
	coffmch_T_cup_stand = rdk.TxyzRxyz_2_Pose([-12.68, 72.0, -290.0, 0, -np.pi/2 , 0])
	cup_tool_center_T_tcp = rdk.TxyzRxyz_2_Pose([-47.0, 0, 186.11, 0, 0, 0]).inv()
	top_of_cup = rdk.TxyzRxyz_2_Pose([85, 0, 0, 0, 0, 0])

	coffmch_T_base = base_T_coffmch()*coffmch_T_cup_stand*cup_tool_center_T_tcp*top_of_cup*rotate_arm_T()

	robot.MoveJ(coffmch_T_base)


def exit_cup_standoff(robot):
	# 
	coffmch_T_cup_stand = rdk.TxyzRxyz_2_Pose([-12.68, 72.0, -290.0, 0, -np.pi/2 , 0])
	cup_tool_center_T_tcp = rdk.TxyzRxyz_2_Pose([-47.0, 0, 186.11, 0, 0, 0]).inv()
	top_of_cup_exit = rdk.TxyzRxyz_2_Pose([85, 0, -80, 0, 0, 0])

	coffmch_T_exit = base_T_coffmch()*coffmch_T_cup_stand*cup_tool_center_T_tcp*top_of_cup_exit*rotate_arm_T()
	robot.MoveJ(coffmch_T_exit)


def coffee_switch(robot, delay):
	# Turn the coffee switch on and off with a delay between

	# Frames
	pointer_T_end_tcp = rdk.TxyzRxyz_2_Pose([0, 0, 102.82, 0, 0, 0]).inv()
	coffmch_T_coffee_switch = rdk.TxyzRxyz_2_Pose([50.67, 35.25, -27.89, 0, np.radians(-90), 0])
	coffee_switch_T_on_apprch = rdk.TxyzRxyz_2_Pose([-7, 0, -15, 0, 0, 0])
	coffee_switch_T_on = rdk.TxyzRxyz_2_Pose([-7, 0, 0, 0, 0, 0])
	coffee_switch_T_off_apprch = rdk.TxyzRxyz_2_Pose([7, 0, -15, 0, 0, 0])
	coffee_switch_T_off = rdk.TxyzRxyz_2_Pose([7, 0, 0, 0, 0, 0])
	
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


def tamper_to_coffee_machine(robot, delay):
	# Move to a position where the portafilter can be manually removed and connect to the coffee machine
	
	# Intermediate positions
	servo_positions = [[-1.936582734206446, -66.74306264032822, -110.53394906188917, 186.1451355960815, -122.1456802642395, 144.7457822986689],
	[-73.42, -66.73, -110.53, 186.14, -122.14, 144.74],
	[-130.34, -79.87, -140.16, 237.81, -138.62, 156.12]]
	
	for pos in servo_positions:
		robot.MoveJ(pos)

	rdk.pause(delay)
