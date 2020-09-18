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


# Grinder 'on' button approach
grinder_T_stand = rdk.TxyzRxyz_2_Pose([-64.42, 89.82, -227.68, 0, np.radians(-90), 0])
r = rdk.TxyzRxyz_2_Pose([0, 102.82 + 20, 0, np.radians(90), 0, 0])
portafilter_stand = base_T_grinder*grinder_T_stand*r
robot.MoveJ(portafilter_stand, blocking=True)

# Grinder 'on' button press
grinder_T_stand = rdk.TxyzRxyz_2_Pose([-64.42, 89.82, -227.68, 0, np.radians(-90), 0])
r = rdk.TxyzRxyz_2_Pose([0, 102.82, 0, np.radians(90), 0, 0])
portafilter_stand = base_T_grinder*grinder_T_stand*r
robot.MoveJ(portafilter_stand, blocking=True)

# Grinder 'on' button release
grinder_T_stand = rdk.TxyzRxyz_2_Pose([-64.42, 89.82, -227.68, 0, np.radians(-90), 0])
r = rdk.TxyzRxyz_2_Pose([0, 102.82 + 20, 0, np.radians(90), 0, 0])
portafilter_stand = base_T_grinder*grinder_T_stand*r
robot.MoveJ(portafilter_stand, blocking=True)


rdk.pause(3)   


# Grinder 'off' button approach
grinder_T_stand = rdk.TxyzRxyz_2_Pose([-80.71, 94.26, -227.68, 0, np.radians(-90), 0])
r = rdk.TxyzRxyz_2_Pose([0, 102.82 + 20, 0, np.radians(90), 0, 0])
portafilter_stand = base_T_grinder*grinder_T_stand*r
robot.MoveJ(portafilter_stand, blocking=True)

# Grinder 'off' button press
grinder_T_stand = rdk.TxyzRxyz_2_Pose([-80.71, 94.26, -227.68, 0, np.radians(-90), 0])
r = rdk.TxyzRxyz_2_Pose([0, 102.82, 0, np.radians(90), 0, 0])
portafilter_stand = base_T_grinder*grinder_T_stand*r
robot.MoveJ(portafilter_stand, blocking=True)

# Grinder 'off' button release
grinder_T_stand = rdk.TxyzRxyz_2_Pose([-80.71, 94.26, -227.68, 0, np.radians(-90), 0])
r = rdk.TxyzRxyz_2_Pose([0, 102.82 + 20, 0, np.radians(90), 0, 0])
portafilter_stand = base_T_grinder*grinder_T_stand*r
robot.MoveJ(portafilter_stand, blocking=True)
