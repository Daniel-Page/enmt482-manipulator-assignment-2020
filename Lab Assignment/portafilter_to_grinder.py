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


# Approach grinder with portafilter
# The location of the stand for the portafilter

# Move to home
robot.MoveJ(target, blocking=True)


# Intermediate point between tool stand and grinder
intermediate_point = rdk.Mat([   [-1.000000,    -0.000000,     0.000000,    14.953000],
    [ -0.000000,     1.000000,    -0.000000,  -304.876000 ],
    [ -0.000000,    -0.000000,    -1.000000,   388.727000] ,
     [ 0.000000,     0.000000,     0.000000,     1.000000 ]])
robot.MoveJ(intermediate_point, blocking=True)


# Approach portafilter tool on the stand
T_portafilter_tool_approach = rdk.TxyzRxyz_2_Pose([-399.59, -55.10, 534.02, 0, np.pi, 0])
robot.MoveJ(T_portafilter_tool_approach, blocking=True)


# Attach portafilter from stand
RDK.RunProgram('Portafilter Tool Attach (Stand)', True)
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())


intermediate_point = rdk.Mat([   [ 0.442774,    -0.896633,    -0.000211,  -249.591000],
    [ -0.896594,    -0.442757,     0.009220,   -95.098000],
    [-0.008360,    -0.003893,    -0.999957,   534.002000],
    [0.000000,     0.000000,     0.000000,     1.000000] ])
robot.MoveJ(intermediate_point, blocking=True)


intermediate_point = rdk.Mat([     [0.442774,    -0.896633,    -0.000211,  -159.591000],
     [-0.896594,    -0.442757,     0.009220,  -195.098000],
     [-0.008360,    -0.003893,    -0.999957,   534.002000],
      [0.000000,     0.000000,     0.000000,     1.000000] ])
robot.MoveJ(intermediate_point, blocking=True)


intermediate_point = rdk.Mat([     [0.442774,    -0.896633,    -0.000211,   -19.591000],
     [-0.896594,    -0.442757,     0.009220,  -305.098000],
    [ -0.008360,    -0.003893,    -0.999957,   534.002000] ,
     [ 0.000000,     0.000000,     0.000000,     1.000000 ]])
robot.MoveJ(intermediate_point, blocking=True)


intermediate_point = rdk.Mat([     [0.442774,    -0.896633,    -0.000211,   230.409000],
     [-0.896594,    -0.442757,     0.009220,  -305.098000],
     [-0.008360,    -0.003893,    -0.999957,   534.002000],
      [0.000000,     0.000000,     0.000000,     1.000000] ])
robot.MoveJ(intermediate_point, blocking=True)


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


grinder_T_stand = rdk.TxyzRxyz_2_Pose([157.61 - 2.3, 0, -250.45 - 2.9, 0, np.radians(-90), 0])
r = rdk.TxyzRxyz_2_Pose([0, 0, 0, 0, np.radians(7.5), 0])
q = rdk.TxyzRxyz_2_Pose([32.0, 0, -27.56, 0, 0, 0])
m = rdk.TxyzRxyz_2_Pose([0, 0, 0,0,0, np.radians(50)])


portafilter_stand = base_T_grinder*grinder_T_stand*r*q*m
robot.MoveJ(portafilter_stand, blocking=True)


# Detach portafilter at the grinder
RDK.RunProgram('Portafilter Tool Detach (Grinder)', True)
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())
