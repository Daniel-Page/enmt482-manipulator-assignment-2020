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
	# Grinder reference frame with respect to the robot base reference frame
	return rdk.Mat([
	    [-7.3289583075E-01, -6.8034087138E-01, 0.0000000000E+00,
	         4.8451000000E+02],
	        [6.8034087138E-01, -7.3289583075E-01, 0.0000000000E+00,
	         -4.2660000000E+02],
	        [0.0000000000E+00, 0.0000000000E+00, 1.0000000000E+00,
	         3.1838000000E+02],
	        [0.0000000000E+00, 0.0000000000E+00, 0.0000000000E+00,
	         1.0000000000E+00]
	])


def rotate_arm_T():
	# Rotate the end effector 50 degees
	return rdk.TxyzRxyz_2_Pose([0, 0, 0,0,0, np.radians(50)])


def home_to_tool_stand_portafilter(robot):
	servo_positions = [[-11.84, -90.0, -89.99, -89.99, 90.0, 0.0],
	[-37.89, -90.0, -89.99, -89.99, 90.0, 0.0],
	[-61.58, -90.0, -89.99, -89.99, 90.0, 0.0],
	[-80.53, -90.0, -89.99, -89.99, 90.0, 0.0],
	[-101.04641114821452, -83.7466574162878, -78.03965611514451, -108.18364791378339, 90.00059210641064, -1.5765964271364985],
	[-127.88997245104923, -84.18972515746033, -76.09053777653044, -109.6696897502238, 90.00001482352836, -1.5697234976294043]]

	for ii in servo_positions:
	    robot.MoveJ(ii, blocking=True)

	# Approach portafilter tool on the stand
	T_portafilter_tool_approach = rdk.TxyzRxyz_2_Pose([-399.59, -55.10, 534.02, 0, np.pi, 0])
	robot.MoveJ(T_portafilter_tool_approach, blocking=True)


def attach_portafilter(robot, RDK, world_frame):
	# Attach portafilter from stand
	RDK.RunProgram('Portafilter Tool Attach (Stand)', True)
	robot.setPoseFrame(world_frame)
	robot.setPoseTool(robot.PoseTool())


def tool_stand_to_grinder_portafilter(robot):
	servo_positions = [[-156.30685678169834, -81.17665202043209, -75.3849671408721, -113.63929262831658, 89.51132826845027, -182.58734428665286],
	[-113.68, -81.17, -75.38, -113.63, 89.51, -182.58],
	[-73.42, -81.17, -75.38, -113.63, 89.51, -182.58],
	[-54.47, -81.17, -75.38, -113.63, 89.51, -182.58],
	[-69.94515534638512, -94.53528110337791, -122.45672642721897, -134.83502387874725, -114.41460247016077, 143.44328994555121],
	[-21.287713061147688, -86.49116618896865, -149.85301316939308, -115.52179562257535, -66.19272457274711, 136.74402036652293]]

	for ii in servo_positions:
		robot.MoveJ(ii, blocking=True)


def grinder_lower_portafilter(robot):
	grinder_T_stand = rdk.TxyzRxyz_2_Pose([157.61, 0, -250.45, 0, np.radians(-90), 0])
	bottom_T_tcp = rdk.TxyzRxyz_2_Pose([-32.0, 0, 27.56, 0, np.radians(-7.5), 0]).inv()

	lower_stand_1_T = rdk.TxyzRxyz_2_Pose([100, 0, -80, 0, np.radians(-20), 0])
	lower_stand_2_T = rdk.TxyzRxyz_2_Pose([45, 0, -20, 0, np.radians(-15), 0])
	lower_stand_3_T = rdk.TxyzRxyz_2_Pose([30, 0, 0, 0, np.radians(-10), 0])
	lower_stand_4_T = rdk.TxyzRxyz_2_Pose([10, 0, 0, 0, np.radians(0), 0])
	
	base_T_lower_stand_1 = base_T_grinder()*grinder_T_stand*bottom_T_tcp*lower_stand_1_T*rotate_arm_T()
	base_T_lower_stand_2 = base_T_grinder()*grinder_T_stand*bottom_T_tcp*lower_stand_2_T*rotate_arm_T()
	base_T_lower_stand_3 = base_T_grinder()*grinder_T_stand*bottom_T_tcp*lower_stand_3_T*rotate_arm_T()
	base_T_lower_stand_4 = base_T_grinder()*grinder_T_stand*bottom_T_tcp*lower_stand_4_T*rotate_arm_T()
	base_T_grinder_stand = base_T_grinder()*grinder_T_stand*bottom_T_tcp*rotate_arm_T()

	robot.MoveJ(base_T_lower_stand_1, blocking=True)
	robot.MoveJ(base_T_lower_stand_2, blocking=True)
	robot.MoveJ(base_T_lower_stand_3, blocking=True)
	robot.MoveJ(base_T_lower_stand_4, blocking=True)
	robot.MoveJ(base_T_grinder_stand, blocking=True)


def detach_portafilter(robot, RDK, world_frame):
	# Detach portafilter at the grinder
	RDK.RunProgram('Portafilter Tool Detach (Grinder)', True)
	robot.setPoseFrame(world_frame)
	robot.setPoseTool(robot.PoseTool())


def grinder_stand_to_tool_stand(robot):
	servo_positions = [[-62.64915538638531, -93.57841025826991, -90.54115226190851, -85.88043747982157, 90.0, 27.350844613614694],
	[-98.24274139471727, -98.31364637061935, -58.35101976985437, -113.33533385952627, 90.00000000000001, -8.242741394717262],
	[-121.38940521009599, -90.3316017809194, -67.04022874234826, -112.62816947673232, 90.0, -31.389405210095983]]

	for ii in servo_positions:
		robot.MoveJ(ii, blocking=True)

	T_grinder_tool_approach = rdk.TxyzRxyz_2_Pose([-441.77, -214.48, 534.24, 0, np.pi, 0])
	robot.MoveJ(T_grinder_tool_approach, blocking=True)


def attach_grinder_tool(robot, RDK, world_frame):
	# Detach portafilter at the grinder
	RDK.RunProgram('Grinder Tool Attach (Stand)', True)
	robot.setPoseFrame(world_frame)
	robot.setPoseTool(robot.PoseTool())


def tool_stand_to_grinder_buttons(robot):
	servo_positions = [[-141.26390939194346, -93.1871920405575, -64.04572790653306, -112.91010981783381, 89.94209367320389, -166.99465128597112],
	[-130.26, -93.18, -64.04, -112.91, 89.94, -166.99],
	[-116.05, -93.18, -64.04, -112.91, 89.94, -166.99],
	[-92.37, -93.18, -64.04, -112.91, 89.94, -166.99],
	[-78.16, -93.18, -64.04, -112.91, 89.94, -166.99],
	[-71.05, -93.18, -64.04, -112.91, 89.94, -166.99],
	[-71.77599132549105, -104.33792246677127, -73.49276992535768, -158.02875321196697, 176.3826039661928, -172.73898890655633]]

	for ii in servo_positions:
		robot.MoveJ(ii, blocking=True)


def press_start_stop_grinder(robot):
	tcp_T_pointer_end = rdk.TxyzRxyz_2_Pose([0, 0, 102.82, 0, 0, 0]).inv()
	button_approach_T = rdk.TxyzRxyz_2_Pose([0, 0, -20, 0, 0, 0])


	grinder_T_on_button = rdk.TxyzRxyz_2_Pose([-64.42, 89.82, -227.68, np.radians(90), 0, np.radians(90)])
	base_T_on_button_apprch = base_T_grinder()*grinder_T_on_button*tcp_T_pointer_end*button_approach_T*rotate_arm_T()
	base_T_on_button = base_T_grinder()*grinder_T_on_button*tcp_T_pointer_end*rotate_arm_T()

	robot.MoveJ(base_T_on_button_apprch, blocking=True) # Grinder 'on' button approach
	robot.MoveJ(base_T_on_button, blocking=True) # Grinder 'on' button press
	robot.MoveJ(base_T_on_button_apprch, blocking=True) # Grinder 'on' button release


	rdk.pause(3) # A 3 second pause while the coffee grinding occurs


	grinder_T_off_button = rdk.TxyzRxyz_2_Pose([-80.71, 94.26, -227.68, np.radians(90), 0, np.radians(90)])
	base_T_off_button_apprch = base_T_grinder()*grinder_T_off_button*tcp_T_pointer_end*button_approach_T*rotate_arm_T()
	base_T_off_button = base_T_grinder()*grinder_T_off_button*tcp_T_pointer_end*rotate_arm_T()

	robot.MoveJ(base_T_off_button_apprch, blocking=True) # Grinder 'off' button approach
	robot.MoveJ(base_T_off_button, blocking=True) # Grinder 'off' button press
	robot.MoveJ(base_T_off_button_apprch, blocking=True) # Grinder 'off' button release


def approach_grinder_lever(robot):
	servo_positions = [[-64.77185023344938, -127.12135786977275, -72.80221506393488, -160.07568512512123, 158.09842468457376, -219.99956055641846],
	[-63.58, -89.11, -99.35, -141.39, 159.28, -219.99],
	[81.30007567320077, -91.60066232480632, 105.61061172183275, -14.009456650110751, 34.17035059387107, 229.9993433666025]]

	for ii in servo_positions:
			robot.MoveJ(ii, blocking=True)


def crank_grinder_lever(robot):
	tcp_T_pointer_end = rdk.TxyzRxyz_2_Pose([-50, 0, 67.06, 0, 0, 0]).inv()
	grinder_T_grinder_lever = rdk.TxyzRxyz_2_Pose([-35.82, 83.80, -153.00, 0, np.radians(-90), np.radians(90)])
	grinder_lever_apprch_T = rdk.TxyzRxyz_2_Pose([0, 0, 10, 0, 0, 0])
	grinder_lever_pull_T = rdk.TxyzRxyz_2_Pose([0, 0, -40, 0, 0, 0])

	base_T_grinder_lever_apprch = base_T_grinder()*grinder_T_grinder_lever*tcp_T_pointer_end*grinder_lever_apprch_T*rotate_arm_T()
	base_T_grinder_lever_pull = base_T_grinder()*grinder_T_grinder_lever*tcp_T_pointer_end*grinder_lever_pull_T*rotate_arm_T()

	robot.MoveJ(base_T_grinder_lever_apprch, blocking=True)
	robot.MoveJ(base_T_grinder_lever_pull, blocking=True)
	robot.MoveJ(base_T_grinder_lever_apprch, blocking=True)
