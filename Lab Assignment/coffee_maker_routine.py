'''
	ENMT482
	Robotics
	Assignment 2
	Coffee Maker Routine
	Daniel Page & Tom Coulson
'''


import robolink as rl    # RoboDK API
import robodk as rdk     # Robot toolbox
import numpy as np


RDK = rl.Robolink()
robot = RDK.Item('UR5')
world_frame = RDK.Item('UR5 Base')
target = RDK.Item('Home')   # existing target in station
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())


# Provided routines
portafilter_attach_stand = RDK.Item('Portafilter Tool Attach (Stand)')
portafilter_detach_stand = RDK.Item('Portafilter Tool Detach (Stand)')
grinder_attach_stand = RDK.Item('Grinder Tool Attach (Stand)')
grinder_detach_stand = RDK.Item('Grinder Tool Detach (Stand)')
portafilter_detach_grinder = RDK.Item('Portafilter Tool Detach (Grinder)')


# Tool stand entry positions
T_cup_tool_approach = rdk.TxyzRxyz_2_Pose([-358.13, 102.68, 532.67, 0, np.pi, 0])


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


# The location of the stand for the portafilter
grinder_T_stand = rdk.TxyzRxyz_2_Pose([157.61, 0, -250.45, 0, -np.pi/2, 0])


########################
# Beginning of routine #
########################


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
portafilter_attach_stand.RunCode()
portafilter_attach_stand.WaitFinished()
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())


# Intermediate point between tool stand and grinder
intermediate_point = rdk.Mat([   [-1.000000,    -0.000000,     0.000000,    14.953000],
    [ -0.000000,     1.000000,    -0.000000,  -304.876000 ],
    [ -0.000000,    -0.000000,    -1.000000,   388.727000] ,
     [ 0.000000,     0.000000,     0.000000,     1.000000 ]])
robot.MoveJ(intermediate_point, blocking=True)



# Approach grinder with portafilter
rotate_portafilter = rdk.TxyzRxyz_2_Pose([0, 0, 0, 0, np.radians(7.5), 0])
change_reference_frame_tcp = rdk.TxyzRxyz_2_Pose([4.71, 0, 144.76, 0, 0, 0])
offset_to_bottom_tool = rdk.TxyzRxyz_2_Pose([32.0, 0, -27.56, 0, 0, 0])
rotate_portafilter_back = rdk.TxyzRxyz_2_Pose([0, 0, 0, 0, np.radians(-7.5), 0])
portafilter_stand = base_T_grinder*grinder_T_stand*rotate_portafilter*change_reference_frame_tcp*offset_to_bottom_tool*rotate_portafilter_back
robot.MoveJ(portafilter_stand, blocking=True)


# Detach portafilter at the grinder
portafilter_detach_grinder.RunCode()
portafilter_detach_grinder.WaitFinished()
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())


# Intermediate point between tool stand and grinder
intermediate_point = rdk.Mat([   [-1.000000,    -0.000000,     0.000000,    14.953000],
    [ -0.000000,     1.000000,    -0.000000,  -304.876000 ],
    [ -0.000000,    -0.000000,    -1.000000,   388.727000] ,
     [ 0.000000,     0.000000,     0.000000,     1.000000 ]])
robot.MoveJ(intermediate_point, blocking=True)


T_grinder_tool_approach = rdk.TxyzRxyz_2_Pose([-441.77, -214.48, 534.24, 0, np.pi, 0])
robot.MoveJ(T_grinder_tool_approach, blocking=True)


# Intermediate point between tool stand and grinder
intermediate_point = rdk.Mat([   [-1.000000,    -0.000000,     0.000000,    14.953000],
    [ -0.000000,     1.000000,    -0.000000,  -304.876000 ],
    [ -0.000000,    -0.000000,    -1.000000,   388.727000] ,
     [ 0.000000,     0.000000,     0.000000,     1.000000 ]])
robot.MoveJ(intermediate_point, blocking=True)


# Attach grinder tool from stand
grinder_attach_stand.RunCode()
grinder_attach_stand.WaitFinished()
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())
