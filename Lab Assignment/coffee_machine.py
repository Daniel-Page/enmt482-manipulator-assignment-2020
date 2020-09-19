import robodk as rdk # Robot toolbox
import numpy as np


def base_T_coffmch():
	return rdk.Mat([[-2.5474282622E-01, -9.6700883785E-01, 0.0000000000E+00,
         -3.5990000000E+02],
        [9.6700883785E-01, -2.5474282622E-01, 0.0000000000E+00,
         -3.8738000000E+02],
        [0.0000000000E+00, 0.0000000000E+00, 1.0000000000E+00,
         3.4124000000E+02],
        [0.0000000000E+00, 0.0000000000E+00, 0.0000000000E+00,
         1.0000000000E+00]])


def coffee_switch(robot):
	coffmch_T_coffee_switch = rdk.TxyzRxyz_2_Pose([50.67, 35.25, -27.89, 0.00, np.radians(-90), 0])
	base_T_coffee_switch = base_T_coffmch()*coffmch_T_coffee_switch
	robot.MoveJ(base_T_coffee_switch, blocking=True)


def hot_water_switch(robot):
	coffmch_T_hot_water_switch = rdk.TxyzRxyz_2_Pose([50.67, 35.25, -61.39, 0.00, np.radians(-90), 0])
	base_T_hot_water_switch = base_T_coffmch()*coffmch_T_hot_water_switch
	robot.MoveJ(base_T_hot_water_switch, blocking=True)


def steam_switch(robot):
	coffmch_T_steam_switch = rdk.TxyzRxyz_2_Pose([50.67, 35.25, -94.89, 0.00, np.radians(-90), 0])
	base_T_steam_switch = base_T_coffmch()*coffmch_T_steam_switch
	robot.MoveJ(base_T_steam_switch, blocking=True)


def power_switch(robot):
	coffmch_T_power_switch = rdk.TxyzRxyz_2_Pose([50.67, 98.75, -27.89, 0, np.radians(-90), 0])
	base_T_power_switch = base_T_coffmch()*coffmch_T_power_switch
	robot.MoveJ(base_T_power_switch, blocking=True)
