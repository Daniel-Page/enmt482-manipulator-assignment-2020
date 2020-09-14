'''
	ENMT482
	Robotics
	Assignment 2
	Coffee Maker Robot
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
portafilter_attach = RDK.Item('Portafilter Tool Attach (Stand)')
portafilter_detach = RDK.Item('Portafilter Tool Detach (Stand)')
grinder_attach = RDK.Item('Grinder Tool Attach (Stand)')
grinder_detach = RDK.Item('Grinder Tool Detach (Stand)')



T_porta_filter_approach = rdk.Mat([
	[-1.0000000e+00,  0.0000000e+00,  1.2246468e-16, -3.9959000e+02],
	[ 0.0000000e+00,  1.0000000e+00,  0.0000000e+00, -5.5100000e+01],
    [-1.2246468e-16,  0.0000000e+00, -1.0000000e+00,  5.3402000e+02],
    [ 0.0000000e+00,  0.0000000e+00,  0.0000000e+00,  1.0000000e+00]
])

base_T_grinder = rdk.Mat([
	[-7.3289583E-01, -6.8034087E-01, 0.0000000E+00,  4.8451000E+02],
	[ 6.8034087E-01, -7.3289583E-01, 0.0000000E+00, -4.2660000E+02],
	[ 0.0000000E+00,  0.0000000E+00, 1.0000000E+00,  3.1838000E+02],
	[ 0.0000000E+00,  0.0000000E+00, 0.0000000E+00,  1.0000000E+00]
])


robot.MoveJ(target, blocking=True)
robot.MoveJ(T_porta_filter_approach, blocking=True)

portafilter_attach.RunCode()
#robot.setPoseFrame(world_frame)
#robot.setPoseTool(robot.PoseTool())

#rdk.pause(5)
#robot.MoveJ(base_T_grinder, blocking=True)
