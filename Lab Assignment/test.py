'''
	ENMT482
	Robotics
	Assignment 2
	Test
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

#robot.MoveL(target, blocking=True)


# Use the RDK Matrix object from to hold pose (its an HT)
base_T_coffmch = rdk.Mat([[-2.54742826e-01, -9.67008838e-01,  0.00000000e+00, -3.59900000e+02],
 [ 9.67008838e-01, -2.54742826e-01,  0.00000000e+00, -3.87380000e+02],
 [ 0.00000000e+00,  0.00000000e+00,  1.00000000e+00,  3.41240000e+02],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

coffmch_T_switch = rdk.Mat([[ 6.123234e-17,  0.000000e+00, -1.000000e+00,  5.067000e+01],
        [ 0.000000e+00,  1.000000e+00,  0.000000e+00,  3.525000e+01],
        [ 1.000000e+00,  0.000000e+00,  6.123234e-17, -9.489000e+01],
        [ 0.000000e+00,  0.000000e+00,  0.000000e+00,  1.000000e+00]])


grinder_P_loc = np.matrix([50,0,0]).T

base_P_loc = base_T_coffmch*coffmch_T_switch



robot.MoveJ(base_P_loc, blocking=True)




#robot.MoveL(pose, blocking=True)


