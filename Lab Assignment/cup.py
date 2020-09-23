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
	servo_positions = [[-147.86, -76.89, -79.98, -113.53, 91.6, -114.61],
	[-112.5, -76.89, -79.98, -113.53, 91.6, -114.61],
	[-73.93, -76.89, -79.98, -113.53, 91.6, -114.61],
	[-73.93, -76.89, -79.98, -155.53, 91.6, -113.61],
	[-73.93, -76.89, -79.98, -155.53, 237.86, -113.61],
	[-73.93, -76.89, -79.98, -186.53, 276.43, -113.61],
	[-76.34898929699695, -80.94252952651527, -149.310818753435, -125.16479212080955, 274.01924318971624, -35.16593220428045]]



	for ii in servo_positions:
		robot.MoveJ(ii, blocking=True)
		
	
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
