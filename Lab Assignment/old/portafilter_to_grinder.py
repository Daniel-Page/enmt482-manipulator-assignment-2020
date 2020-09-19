'''
	ENMT482
	Robotics
	Assignment 2
	Coffee Maker Routine
	Daniel Page & Tom Coulson
'''


# Right click on provided scripts and click run on robot to work
# Double click disconnect
# Get robot position


import robolink as rl    # RoboDK API
import robodk as rdk     # Robot toolbox
import numpy as np


RDK = rl.Robolink()
robot = RDK.Item('UR5')
world_frame = RDK.Item('UR5 Base')
target = RDK.Item('Home')   # existing target in station
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())


########################
# Beginning of routine #
########################

# Move to home
robot.MoveJ(target, blocking=True)

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

# Attach portafilter from stand
RDK.RunProgram('Portafilter Tool Attach (Stand)', True)
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())

servo_positions = [[-156.30685678169834, -81.17665202043209, -75.3849671408721, -113.63929262831658, 89.51132826845027, -182.58734428665286],
[-113.68, -81.17, -75.38, -113.63, 89.51, -182.58],
[-73.42, -81.17, -75.38, -113.63, 89.51, -182.58],
[-54.47, -81.17, -75.38, -113.63, 89.51, -182.58],
[-69.94515534638512, -94.53528110337791, -122.45672642721897, -134.83502387874725, -114.41460247016077, 143.44328994555121],
[-21.287713061147688, -86.49116618896865, -149.85301316939308, -115.52179562257535, -66.19272457274711, 136.74402036652293]]

for ii in servo_positions:
    robot.MoveJ(ii, blocking=True)

# Grinder reference frame with respect to the robot base reference frame
base_T_grinder = rdk.Mat([
    [-7.3289583075E-01, -6.8034087138E-01, 0.0000000000E+00,
         4.8451000000E+02],
        [6.8034087138E-01, -7.3289583075E-01, 0.0000000000E+00,
         -4.2660000000E+02],
        [0.0000000000E+00, 0.0000000000E+00, 1.0000000000E+00,
         3.1838000000E+02],
        [0.0000000000E+00, 0.0000000000E+00, 0.0000000000E+00,
         1.0000000000E+00]
])

r = rdk.TxyzRxyz_2_Pose([80, 0, 40, 0, np.radians(-10), 0])
grinder_T_stand = rdk.TxyzRxyz_2_Pose([157.61, 0, -250.45, np.radians(-50), np.radians(-90), 0])
portafilter_stand = base_T_grinder*r*grinder_T_stand
robot.MoveJ(portafilter_stand, blocking=True)

r = rdk.TxyzRxyz_2_Pose([0, 0, 40, 0, np.radians(-10), 0])
grinder_T_stand = rdk.TxyzRxyz_2_Pose([157.61, 0, -250.45, np.radians(-50), np.radians(-90), 0])
portafilter_stand = base_T_grinder*r*grinder_T_stand
robot.MoveJ(portafilter_stand, blocking=True)

r = rdk.TxyzRxyz_2_Pose([10, 0, 40, 0, np.radians(-2), 0])
grinder_T_stand = rdk.TxyzRxyz_2_Pose([157.61, 0, -250.45, np.radians(-50), np.radians(-90), 0])
portafilter_stand = base_T_grinder*r*grinder_T_stand
robot.MoveJ(portafilter_stand, blocking=True)

x_offset = 0#-2.3
z_offset = 0#-2.9
grinder_T_stand = rdk.TxyzRxyz_2_Pose([157.61 + x_offset, 0, -250.45 + z_offset, 0, np.radians(-90), 0])
bottom_T_tcp = rdk.TxyzRxyz_2_Pose([-32.0, 0, 27.56, 0, np.radians(-7.5), 0]).inv()
rotate_arm_T = rdk.TxyzRxyz_2_Pose([0, 0, 0,0,0, np.radians(50)])
portafilter_stand = base_T_grinder*grinder_T_stand*bottom_T_tcp*rotate_arm_T
robot.MoveJ(portafilter_stand, blocking=True)

# Detach portafilter at the grinder
RDK.RunProgram('Portafilter Tool Detach (Grinder)', True)
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())

for ii in reversed(servo_positions):
    robot.MoveJ(ii, blocking=True)

# Detach portafilter at the grinder
RDK.RunProgram('Grinder Tool Attach (Stand)', True)
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())
