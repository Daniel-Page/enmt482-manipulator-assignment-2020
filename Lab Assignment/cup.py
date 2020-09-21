'''
	ENMT482
	Robotics
	Assignment 2
	Cup Functions
	Daniel Page & Tom Coulson
'''


import robodk as rdk # Robot toolbox
import numpy as np

def rotate_arm_T():
	# Rotate the end effector 50 degees
	return rdk.TxyzRxyz_2_Pose([0, 0, 0,0,0, np.radians(50)])


def home_to_tool_stand_cup(robot):
	T_cup_tool_approach = rdk.TxyzRxyz_2_Pose([-358.13, 102.68, 532.67, 0, np.pi, 0])
	robot.MoveJ(T_cup_tool_approach, blocking=True)


def attach_cup_tool(robot, RDK, world_frame):
	RDK.RunProgram('Cup Tool Attach (Stand)', True)
	robot.setPoseFrame(world_frame)
	robot.setPoseTool(robot.PoseTool())


def tool_stand_to_cups(robot):
	pass


def lower_tool_to_cups(robot):
	base_T_cup = rdk.TxyzRxyz_2_Pose([1.92, -595.89, -20.0, 0, 0, 0])
	cup_bottom_T_top_edge = rdk.TxyzRxyz_2_Pose([0, 0, 147.25, np.pi/2, 0, np.pi/2])
	cup_tool_center_T_tcp = rdk.TxyzRxyz_2_Pose([-47.0, 0, 186.11, 0, 0, 0]).inv()
	cups_apprch_T = rdk.TxyzRxyz_2_Pose([100, 0, 0, 0, 0, 0])
	
	cup_apprch_T = base_T_cup*cup_bottom_T_top_edge*cup_tool_center_T_tcp*cups_apprch_T*rotate_arm_T()
	base_T_cup = base_T_cup*cup_bottom_T_top_edge*cup_tool_center_T_tcp*rotate_arm_T()
	robot.MoveJ(cup_apprch_T, blocking=True)
	robot.MoveJ(base_T_cup, blocking=True)


def cup_tool_open(robot, RDK, world_frame):
	RDK.RunProgram('Cup Tool Open', True)
	robot.setPoseFrame(world_frame)
	robot.setPoseTool(robot.PoseTool())


def cup_tool_close(robot, RDK, world_frame):
	RDK.RunProgram('Cup Tool Close', True)
	robot.setPoseFrame(world_frame)
	robot.setPoseTool(robot.PoseTool())
