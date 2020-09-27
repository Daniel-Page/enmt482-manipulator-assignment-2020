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
	#
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
	#
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
	top_of_cup = rdk.TxyzRxyz_2_Pose([-80, 0, 0, 0, 0, 0]).inv()

	coffmch_T_base = base_T_coffmch()*coffmch_T_cup_stand*cup_tool_center_T_tcp*top_of_cup*rotate_arm_T()
	
	robot.MoveJ(coffmch_T_base)


def coffee_switch(robot):
	#
	coffmch_T_coffee_switch = rdk.TxyzRxyz_2_Pose([50.67, 35.25, -27.89, 0.00, np.radians(-90), 0])
	base_T_coffee_switch = base_T_coffmch()*coffmch_T_coffee_switch
	robot.MoveJ(base_T_coffee_switch)


def hot_water_switch(robot):
	#
	coffmch_T_hot_water_switch = rdk.TxyzRxyz_2_Pose([50.67, 35.25, -61.39, 0.00, np.radians(-90), 0])
	base_T_hot_water_switch = base_T_coffmch()*coffmch_T_hot_water_switch
	robot.MoveJ(base_T_hot_water_switch)


def steam_switch(robot):
	#
	coffmch_T_steam_switch = rdk.TxyzRxyz_2_Pose([50.67, 35.25, -94.89, 0.00, np.radians(-90), 0])
	base_T_steam_switch = base_T_coffmch()*coffmch_T_steam_switch
	robot.MoveJ(base_T_steam_switch)


def power_switch(robot):
	#
	coffmch_T_power_switch = rdk.TxyzRxyz_2_Pose([50.67, 98.75, -27.89, 0, np.radians(-90), 0])
	base_T_power_switch = base_T_coffmch()*coffmch_T_power_switch
	robot.MoveJ(base_T_power_switch)
