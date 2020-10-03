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
	return rdk.TxyzRxyz_2_Pose([0, 0, 0, 0, 0, np.radians(50)])


def home_to_tool_stand_cup(robot):
	# Move from home to the tool stand entry point for the cup tool
	
	# Intermediate points
	servo_positions = [[-21.0, -90.0, -89.99, -89.99, 90.0, 0.0],
	[-58.0, -90.0, -89.99, -89.99, 90.0, 0.0],
	[-81.0, -90.0, -89.99, -89.99, 90.0, 0.0],
	[-103.21, -70.17, -106.13, -93.67, 90.0, -2.2],
	[-150.40207168091155, -65.50850075811555, -108.88110432663983, -95.58156804622953, 90.00763691373358, -16.39194402880508],
	[-168.40007259842378, -68.46156438080033, -92.21709711045362, -109.28130149736107, 89.99998780822622, -16.390173975466837]]

	for pos in servo_positions:
		robot.MoveJ(pos)

	# Approach the cup tool on the stand
	T_cup_tool_approach = rdk.TxyzRxyz_2_Pose([-358.13, 102.68, 532.67, 0, np.pi, np.radians(-120)])
	robot.MoveL(T_cup_tool_approach)


def portafilter_handover_to_cup_tool(robot):
	# Move the the stand in preparation to connect to the cup tool

	# Intermediate positions
	servo_positions = [[-130.26, -68.84, -127.0, -155.27, -122.13, 144.72],
	[-135.24024303382987, -74.08103134438778, -123.5715908068476, -155.55947122022786, -20.38402226443112, 146.92728508746526],
	[-135.2398519732695, -71.59932128470875, -104.53689639115781, -96.3640054971551, 91.48961197494872, 143.76025136988378],
	[-139.14051301308413, -80.54292969886068, -97.90245875402228, -94.15030314253825, 91.31613885461164, 139.8622793418919],
	[-151.58, -75.73, -79.79, -114.46, 90.0, 271.04]]
	
	for pos in servo_positions:
		robot.MoveJ(pos)

	# Approach the cup tool on the stand
	T_cup_tool_approach = rdk.TxyzRxyz_2_Pose([-358.13, 102.68, 532.67, 0, np.pi, 0])
	robot.MoveL(T_cup_tool_approach)


def attach_cup_tool(robot, RDK, world_frame, master_tool):
	# Run program to attach the cup tool
	RDK.RunProgram("Cup Tool Attach (Stand)", True)
	robot.setPoseFrame(world_frame)
	robot.setPoseTool(master_tool) # This is used as the program does not reset to the tcp


def tool_stand_to_cups(robot):
	# Move from the tool stand to the vicinity of the stack of cups

	# Intermediate points
	servo_positions = [[-152.54, -76.89, -79.98, -113.53, 91.6, -114.61],
	[-118.54, -76.89, -79.98, -113.53, 91.6, -114.61],
	[-100.54, -76.89, -79.98, -113.53, 91.6, -114.61],
	[-103.78610788278438, -73.28966236992329, -115.56459101758284, -73.77534386433494, 189.7941758539359, -95.5636146444791],
	[-108.05814789749287, -64.17890550395677, -121.08257313172102, -161.21213996664122, 246.5273657321103, -94.50120064782078],
	[-61.23, -55.99, -145.79, -152.8, 292.17, -47.45]]

	for pos in servo_positions:
		robot.MoveJ(pos)
		

def cup_tool_open(robot, RDK, world_frame):
	# Run program to open the cup tool 
	RDK.RunProgram("Cup Tool Open", True)
	robot.setPoseFrame(world_frame)
	robot.setPoseTool(robot.PoseTool())

	
def skewer_cup(robot):
	# Grab a cup from the stack (the 20th cup high)

	# Frames
	base_T_cup = rdk.TxyzRxyz_2_Pose([1.92, -595.89, -20.0, 0, 0, 0])
	cup_T_middle_point = rdk.TxyzRxyz_2_Pose([0, 0, 147.25, np.pi/2, 0, -np.pi/2])
	cup_tool_center_T_tcp = rdk.TxyzRxyz_2_Pose([-47, 0, 186.11, 0, 0, 0]).invH()
	middle_point_T_cups_apprch = rdk.TxyzRxyz_2_Pose([0, 0, -60, 0, 0, 0])
	
	base_T_cups_apprch = base_T_cup*cup_T_middle_point*middle_point_T_cups_apprch*cup_tool_center_T_tcp*rotate_arm_T()
	base_T_cups = base_T_cup*cup_T_middle_point*cup_tool_center_T_tcp*rotate_arm_T()

	# Move towards the stack of cups horizontally
	robot.MoveJ(base_T_cups_apprch)
	robot.MoveJ(base_T_cups)


def cup_tool_close(robot, RDK, world_frame):
	# Run program to close the cup tool
	RDK.RunProgram("Cup Tool Close", True)
	robot.setPoseFrame(world_frame)
	robot.setPoseTool(robot.PoseTool())


def lift_cup_from_stack(robot):
	# Lift the cup above the stack
	
	# Frames
	base_T_cup = rdk.TxyzRxyz_2_Pose([1.92, -595.89, -20, 0, 0, 0])
	cup_T_middle_point = rdk.TxyzRxyz_2_Pose([0, 0, 147.25, np.pi/2, 0, -np.pi/2])
	cup_tool_center_T_tcp = rdk.TxyzRxyz_2_Pose([-47, 0, 186.11, 0, 0, 0]).invH()
	middle_point_T_lift = rdk.TxyzRxyz_2_Pose([-150, 0, 0, 0, 0, 0])

	base_T_lift = base_T_cup*cup_T_middle_point*middle_point_T_lift*cup_tool_center_T_tcp*rotate_arm_T()
	
	robot.MoveJ(base_T_lift)


def detach_cup_tool(robot, RDK, world_frame):
	# Run program to detach the cup tool
	
	RDK.RunProgram("Cup Tool Detach (Stand)", True)
	robot.setPoseFrame(world_frame)