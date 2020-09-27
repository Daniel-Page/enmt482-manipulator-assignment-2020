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
	servo_positions = [[-21.0, -90.0, -89.99, -89.99, 90.0, 0.0],
	[-58.0, -90.0, -89.99, -89.99, 90.0, 0.0],
	[-81.0, -90.0, -89.99, -89.99, 90.0, 0.0],
	[-103.21, -70.17, -106.13, -93.67, 90.0, -2.2],
	[-150.40207168091155, -65.50850075811555, -108.88110432663983, -95.58156804622953, 90.00763691373358, -16.39194402880508],
	[-168.40007259842378, -68.46156438080033, -92.21709711045362, -109.28130149736107, 89.99998780822622, -16.390173975466837]]

	for ii in servo_positions:
		robot.MoveJ(ii)

	T_cup_tool_approach = rdk.TxyzRxyz_2_Pose([-358.13, 102.68, 532.67, 0, np.pi, 0])
	robot.MoveL(T_cup_tool_approach, blocking=True)


def attach_cup_tool(robot, RDK, world_frame, master_tool):
	RDK.RunProgram('Cup Tool Attach (Stand)', True)
	robot.setPoseFrame(world_frame)
	robot.setPoseTool(master_tool)


def tool_stand_to_cups(robot):
	servo_positions = [[-152.54, -76.89, -79.98, -113.53, 91.6, -114.61],
	[-118.54, -76.89, -79.98, -113.53, 91.6, -114.61],
	[-100.54, -76.89, -79.98, -113.53, 91.6, -114.61],
	[-103.78610788278438, -73.28966236992329, -115.56459101758284, -73.77534386433494, 189.7941758539359, -95.5636146444791],
	[-108.05814789749287, -64.17890550395677, -121.08257313172102, -161.21213996664122, 246.5273657321103, -94.50120064782078],
	[-61.23, -55.99, -145.79, -152.8, 292.17, -47.45]]

	for ii in servo_positions:
		robot.MoveJ(ii, blocking=True)
		

def cup_tool_open(robot, RDK, world_frame):
	RDK.RunProgram('Cup Tool Open', True)
	robot.setPoseFrame(world_frame)
	robot.setPoseTool(robot.PoseTool())

	
def skewer_cup(robot):
	base_T_cup = rdk.TxyzRxyz_2_Pose([1.92, -595.89, -20.0, 0, 0, 0])
	cup_bottom_T_top_edge = rdk.TxyzRxyz_2_Pose([0, 0, 147.25, np.pi/2, 0, -np.pi/2])
	cup_tool_center_T_tcp = rdk.TxyzRxyz_2_Pose([-47.0, 0, 186.11, 0, 0, 0]).inv()
	cups_apprch_T = rdk.TxyzRxyz_2_Pose([0, 0, -60, 0, 0, 0])
	
	test = base_T_cup*cup_bottom_T_top_edge*cup_tool_center_T_tcp*cups_apprch_T*rotate_arm_T()

	test1 = base_T_cup*cup_bottom_T_top_edge*cup_tool_center_T_tcp*rotate_arm_T()

	robot.MoveJ(test, blocking=True)
	robot.MoveJ(test1, blocking=True)


def cup_tool_close(robot, RDK, world_frame):
	RDK.RunProgram('Cup Tool Close', True)
	robot.setPoseFrame(world_frame)
	robot.setPoseTool(robot.PoseTool())


def lift_cup_from_stack(robot):
	base_T_cup = rdk.TxyzRxyz_2_Pose([1.92, -595.89, -20.0, 0, 0, 0])
	cup_bottom_T_top_edge = rdk.TxyzRxyz_2_Pose([0, 0, 147.25, np.pi/2, 0, -np.pi/2])
	cup_tool_center_T_tcp = rdk.TxyzRxyz_2_Pose([-47.0, 0, 186.11, 0, 0, 0]).inv()
	
	cups_apprch_T = rdk.TxyzRxyz_2_Pose([-150, 0, 0, 0, 0, 0])

	test = base_T_cup*cup_bottom_T_top_edge*cup_tool_center_T_tcp*cups_apprch_T*rotate_arm_T()
	robot.MoveJ(test, blocking=True)

