'''
	ENMT482
	Robotics
	Assignment 2
	Grinder Functions
	Daniel Page & Tom Coulson
'''


import robodk as rdk # Robot toolbox
import numpy as np


def base_T_grinder():
	# Grinder origin in the base reference frame
	base_P_grinder = np.matrix([484.51, -426.60, 318.38]).T

	# Grinder additional point in the base reference frame
	base_P_adpnt = np.matrix([369.74, -320.06, 67.83]).T

	# The change in coordinates with respect to the base
	dspmt_vector = base_P_adpnt - base_P_grinder

	dspmt_xy_vector = [float(dspmt_vector[0]), float(dspmt_vector[1])] # The displacement in terms of x and y

	theta = np.arctan2(dspmt_xy_vector[1],dspmt_xy_vector[0])

	# Coffee grinder reference frame with respect to the robot base reference frame
	return rdk.TxyzRxyz_2_Pose([base_P_grinder[0], base_P_grinder[1], base_P_grinder[2], 0, 0, theta])


def rotate_arm_T():
	# Rotate the end effector 50 degees
	return rdk.TxyzRxyz_2_Pose([0, 0, 0, 0, 0, np.radians(50)])


def home_to_tool_stand_portafilter(robot):
	# Move from the home position to entry point for the portafilter on the tool stand
	
	# Intermediate points
	servo_positions = [[-11.84, -90.0, -89.99, -89.99, 90.0, 0.0],
	[-37.89, -90.0, -89.99, -89.99, 90.0, 0.0],
	[-61.58, -90.0, -89.99, -89.99, 90.0, 0.0],
	[-80.53, -90.0, -89.99, -89.99, 90.0, 0.0],
	[-101.04641114821452, -83.7466574162878, -78.03965611514451, -108.18364791378339, 90.00059210641064, -1.5765964271364985],
	[-127.88997245104923, -84.18972515746033, -76.09053777653044, -109.6696897502238, 90.00001482352836, -1.5697234976294043]]

	for pos in servo_positions:
		robot.MoveJ(pos)

	# Approach the portafilter tool on the tool stand
	base_T_portafilter_tool_approach = rdk.TxyzRxyz_2_Pose([-399.59, -55.1, 534.02, 0, np.pi, 0])
	robot.MoveJ(base_T_portafilter_tool_approach)


def attach_portafilter(robot, RDK, world_frame):
	# Run program to attach portafilter
	
	RDK.RunProgram("Portafilter Tool Attach (Stand)", True)
	robot.setPoseFrame(world_frame)
	robot.setPoseTool(robot.PoseTool())


def tool_stand_to_grinder_portafilter(robot):
	# Move the portafilter from the tool stand to the coffee grinder
	
	# Intermediate points
	servo_positions = [[-113.68, -81.17, -75.38, -113.63, 89.51, -182.58],
	[-73.42, -81.17, -75.38, -113.63, 89.51, -182.58],
	[-54.47, -81.17, -75.38, -113.63, 89.51, -182.58],
	[-69.94515534638512, -94.53528110337791, -122.45672642721897, -134.83502387874725, -114.41460247016077, 143.44328994555121],
	[-21.287713061147688, -86.49116618896865, -149.85301316939308, -115.52179562257535, -66.19272457274711, 136.74402036652293]]

	for pos in servo_positions:
		robot.MoveJ(pos)


def place_portafilter(robot):
	# Carefully place the portafilter into position on the coffee grinder

	# Frames
	grinder_T_stand = rdk.TxyzRxyz_2_Pose([157.61, 0, -250.45, 0, np.radians(-90), 0])
	bottom_T_tcp = rdk.TxyzRxyz_2_Pose([-32, 0, 27.56, 0, 0, 0]).invH()
	angled_bottom_T_tcp = rdk.TxyzRxyz_2_Pose([-32, 0, 27.56, 0, np.radians(-7.5), 0]).invH()
	stand_T_pos_1  = rdk.TxyzRxyz_2_Pose([10, 0, -150, 0, 0, 0])
	stand_T_pos_2 = rdk.TxyzRxyz_2_Pose([10, 0, 0, 0, 0, 0])

	base_T_pos_1 = base_T_grinder()*grinder_T_stand*stand_T_pos_1*bottom_T_tcp*rotate_arm_T()
	base_T_pos_2 = base_T_grinder()*grinder_T_stand*stand_T_pos_2*bottom_T_tcp*rotate_arm_T()
	base_T_grinder_stand = base_T_grinder()*grinder_T_stand*angled_bottom_T_tcp*rotate_arm_T()

	# Move the portafilter horizontally towards the fork
	robot.MoveJ(base_T_pos_1) 
	robot.MoveJ(base_T_pos_2)
	
	# Lower tcp end onto ball
	robot.MoveJ(base_T_grinder_stand)



def detach_portafilter(robot, RDK, world_frame):
	# Run program to detach the portafilter and backoff the stand

	RDK.RunProgram("Portafilter Tool Detach (Grinder)", True)
	robot.setPoseFrame(world_frame)
	robot.setPoseTool(robot.PoseTool())
	
	grinder_T_stand = rdk.TxyzRxyz_2_Pose([157.61, 0, -250.45, 0, np.radians(-90), 0])
	angled_bottom_T_tcp = rdk.TxyzRxyz_2_Pose([-32, 0, 27.56, 0, np.radians(-7.5), 0]).invH()
	angled_bottom_T_stand_backoff = rdk.TxyzRxyz_2_Pose([0, 0, -60, 0, 0, 0])

	base_T_stand_backoff = base_T_grinder()*grinder_T_stand*angled_bottom_T_tcp*angled_bottom_T_stand_backoff*rotate_arm_T()
	robot.MoveJ(base_T_stand_backoff)



def stand_to_tool_stand(robot):
	# Move from the coffee grinder to the entry point for the grinder tool on the tool stand
	
	# Intermediate points
	servo_positions = [[-62.64915538638531, -93.57841025826991, -90.54115226190851, -85.88043747982157, 90.0, 27.350844613614694],
	[-98.24274139471727, -98.31364637061935, -58.35101976985437, -113.33533385952627, 90.00000000000001, -8.242741394717262],
	[-121.38940521009599, -90.3316017809194, -67.04022874234826, -112.62816947673232, 90.0, -31.389405210095983]]

	for pos in servo_positions:
		robot.MoveJ(pos)

	# Approach the grinder tool on the stand
	base_T_grinder_tool_approach = rdk.TxyzRxyz_2_Pose([-441.77, -214.48, 534.24, 0, np.pi, 0])
	robot.MoveJ(base_T_grinder_tool_approach)


def attach_grinder_tool(robot, RDK, world_frame):
	# Run program to attach grinder tool

	RDK.RunProgram("Grinder Tool Attach (Stand)", True)
	robot.setPoseFrame(world_frame)
	robot.setPoseTool(robot.PoseTool())


def tool_stand_to_grinder_buttons(robot):
	# Move from the tool stand to the buttons on the side of the coffee grinder
	
	# Intermediate points
	servo_positions = [[-141.26390939194346, -93.1871920405575, -64.04572790653306, -112.91010981783381, 89.94209367320389, -166.99465128597112],
	[-130.26, -93.18, -64.04, -112.91, 89.94, -166.99],
	[-116.05, -93.18, -64.04, -112.91, 89.94, -166.99],
	[-92.37, -93.18, -64.04, -112.91, 89.94, -166.99],
	[-78.16, -93.18, -64.04, -112.91, 89.94, -166.99],
	[-71.05, -93.18, -64.04, -112.91, 89.94, -166.99],
	[-71.77599132549105, -104.33792246677127, -73.49276992535768, -158.02875321196697, 176.3826039661928, -172.73898890655633]]

	for pos in servo_positions:
		robot.MoveJ(pos)


def press_start_stop_grinder(robot, delay):
	# Press the start button, pause [arg 2] seconds, then press the stop button
	
	# Frames
	pointer_T_end_tcp = rdk.TxyzRxyz_2_Pose([0, 0, 102.82, 0, 0, 0]).invH()
	button_approach_T = rdk.TxyzRxyz_2_Pose([0, 0, -20, 0, 0, 0])
	grinder_T_on_button = rdk.TxyzRxyz_2_Pose([-64.42, 89.82, -227.68, np.radians(90), 0, np.radians(90)])
	
	base_T_on_button_apprch = base_T_grinder()*grinder_T_on_button*pointer_T_end_tcp*button_approach_T*rotate_arm_T()
	base_T_on_button = base_T_grinder()*grinder_T_on_button*pointer_T_end_tcp*rotate_arm_T()
	grinder_T_off_button = rdk.TxyzRxyz_2_Pose([-80.71, 94.26, -227.68, np.radians(90), 0, np.radians(90)])
	base_T_off_button_apprch = base_T_grinder()*grinder_T_off_button*pointer_T_end_tcp*button_approach_T*rotate_arm_T()
	base_T_off_button = base_T_grinder()*grinder_T_off_button*pointer_T_end_tcp*rotate_arm_T()

	robot.MoveJ(base_T_on_button_apprch) # Grinder 'on' button approach
	robot.MoveJ(base_T_on_button) # Grinder 'on' button press
	robot.MoveJ(base_T_on_button_apprch) # Grinder 'on' button release

	rdk.pause(delay) # A [arg 2] second pause while the coffee grinding occurs

	robot.MoveJ(base_T_off_button_apprch) # Grinder 'off' button approach
	robot.MoveJ(base_T_off_button) # Grinder 'off' button press
	robot.MoveJ(base_T_off_button_apprch) # Grinder 'off' button release


def approach_grinder_lever(robot):
	# Move into a position where the lever of the coffee grinder can be cranked

	# Intermediate points
	servo_positions = [[-64.77185023344938, -127.12135786977275, -72.80221506393488, -160.07568512512123, 158.09842468457376, -219.99956055641846],
	[-63.58, -89.11, -99.35, -141.39, 159.28, -219.99],
	[81.30007567320077, -91.60066232480632, 105.61061172183275, -14.009456650110751, 34.17035059387107, 229.9993433666025]]

	for pos in servo_positions:
		robot.MoveJ(pos)


def crank_grinder_lever(robot, angle):
	# Crank the lever [arg 2] degrees, then returns to its previous position

	# Tool frame transform
	pointer_end_T_tcp = rdk.TxyzRxyz_2_Pose([-50, 0, 67.06, 0, 0, 0]).invH()

	init_ang = np.arctan2(83.80, -35.82) # Initial angle of the lever
	radius = np.sqrt((-35.82)**2 + 83.80**2) # The radius of the lever
	
	# Crank the lever
	for rot in range(-10, angle, 10):
		theta = init_ang - np.radians(rot)
		x = np.cos(theta)*radius # The x coordinate of the point on the circle
		y = np.sin(theta)*radius # The y coordinate of the point on the circle
		m = x/y # Gradient (y is opposite to the conventional direction so the expression is not -ve)
		ang_axis_grad = np.arctan2(m, 1) # The angle between the tangent and positive x axis

		grinder_T_tan_point = rdk.TxyzRxyz_2_Pose([x, y, -153, 0, np.radians(-90), np.radians(90)])
		grinder_T_tan_angle = rdk.TxyzRxyz_2_Pose([0, 0, 0, 0, ang_axis_grad, 0])
		
		base_T_grinder_lever_apprch = base_T_grinder()*grinder_T_tan_point*grinder_T_tan_angle*pointer_end_T_tcp*rotate_arm_T()
		
		robot.MoveJ(base_T_grinder_lever_apprch)

	# Uncrank the lever
	for rot in range(angle, -15, -10):
		theta = init_ang - np.radians(rot)
		x = np.cos(theta)*radius
		y = np.sin(theta)*radius
		m = x/y # Gradient (y is opposite to the conventional direction so the expression is not -ve)
		ang_axis_grad = np.arctan2(m, 1) # The angle between the tangent and positive x axis
		
		grinder_T_tan_point = rdk.TxyzRxyz_2_Pose([x, y, -153, 0, np.radians(-90), np.radians(90)])
		grinder_T_tan_angle = rdk.TxyzRxyz_2_Pose([0, 0, 0, 0, ang_axis_grad, 0])
		
		base_T_grinder_lever_apprch = base_T_grinder()*grinder_T_tan_point*grinder_T_tan_angle*pointer_end_T_tcp*rotate_arm_T()
		
		robot.MoveJ(base_T_grinder_lever_apprch)


def lever_to_tool_stand(robot):
	# Move from the coffee grinder lever position to the grinder tool entry point
	
	# Intermediate points
	servo_positions = [[99.64, -66.44, 100.5, -34.05, 64.25, 229.99],
	[99.6398682947346, -74.87436299620246, 86.7098033574326, -11.825573063211925, 64.24948630695674, 229.9900393663943],
	[70.71, -75.98, 73.43, 29.55, 71.25, 229.99],
	[-86.79, -87.22, -68.46, -114.29, 90.0, -56.28]]

	for pos in servo_positions:
		robot.MoveJ(pos)

	# Approach the grinder tool on the tool stand
	base_T_grinder_tool_approach = rdk.TxyzRxyz_2_Pose([-441.77, -214.48, 534.24, 0, np.pi, 0])
	robot.MoveJ(base_T_grinder_tool_approach)


def detach_grinder_tool(robot, RDK, world_frame):
	# Run program to detach the grinder tool

	RDK.RunProgram("Grinder Tool Detach (Stand)", True)
	robot.setPoseFrame(world_frame)
	robot.setPoseTool(robot.PoseTool())


def grinder_portafilter_reapprch(robot):
	# Approach the coffee grinder stand in preparation to connect to the portafilter
	
	# Frames
	grinder_T_stand = rdk.TxyzRxyz_2_Pose([157.61, 0, -250.45, 0, np.radians(-90), 0])
	angled_bottom_T_tcp = rdk.TxyzRxyz_2_Pose([-32, 0, 27.56, 0, np.radians(-7.5), 0]).invH()
	stand_T_stand_apprch = rdk.TxyzRxyz_2_Pose([0, 0, 80, 0, 0, 0]).invH()
	base_T_stand_apprch = base_T_grinder()*grinder_T_stand*angled_bottom_T_tcp*stand_T_stand_apprch*rotate_arm_T()

	robot.MoveJ(base_T_stand_apprch)


def reattach_portafilter(robot, RDK, world_frame):
	# Run program to reattach the portafilter

	RDK.RunProgram("Portafilter Tool Attach (Grinder)", True)
	robot.setPoseFrame(world_frame)
	robot.setPoseTool(robot.PoseTool())

	# Frames
	grinder_T_stand = rdk.TxyzRxyz_2_Pose([157.61, 0, -250.45, 0, np.radians(-90), 0])
	bottom_T_tcp = rdk.TxyzRxyz_2_Pose([-32, 0, 27.56, 0, 0, 0]).invH()
	angled_bottom_T_tcp = rdk.TxyzRxyz_2_Pose([-32, 0, 27.56, 0, np.radians(-7.5), 0]).invH()
	stand_T_pos_1  = rdk.TxyzRxyz_2_Pose([10, 0, -80, 0, 0, 0])
	stand_T_pos_2 = rdk.TxyzRxyz_2_Pose([10, 0, 0, 0, 0, 0])

	base_T_pos_1 = base_T_grinder()*grinder_T_stand*stand_T_pos_1*bottom_T_tcp*rotate_arm_T()
	base_T_pos_2 = base_T_grinder()*grinder_T_stand*stand_T_pos_2*bottom_T_tcp*rotate_arm_T()
	base_T_grinder_stand = base_T_grinder()*grinder_T_stand*angled_bottom_T_tcp*rotate_arm_T()
	
	# Move the portafilter horizontally towards the fork
	robot.MoveJ(base_T_pos_2) 
	robot.MoveJ(base_T_pos_1) 
