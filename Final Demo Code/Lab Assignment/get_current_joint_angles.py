'''
	ENMT482
	Robotics
	Assignment 2
	Get Current Joint Angles
	Daniel Page & Tom Coulson
'''


# Running this script appends an array to a text file
# It is designed to be run mutiple time to generate intermediate points


import robolink as rl    # RoboDK API
import robodk as rdk     # Robot toolbox
import numpy as np


RDK = rl.Robolink()
robot = RDK.Item('UR5')
world_frame = RDK.Item('UR5 Base')
target = RDK.Item('Home')   # existing target in station
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())


text_file = open("joints.txt", "a")
text_file.write('{},\n'.format(robot.Joints().tolist()))
print('{},\n'.format(robot.Joints().tolist()))
text_file.close()
