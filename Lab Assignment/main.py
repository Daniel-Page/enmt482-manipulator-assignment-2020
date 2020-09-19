'''
	ENMT482
	Robotics
	Assignment 2
	Coffee Maker Routine
	Daniel Page & Tom Coulson
'''


# Right click on provided scripts and click run on robot to work
# Double click disconnect
# Get robot position to show actual robot position on the model


import robolink as rl # RoboDK API
import robodk as rdk  # Robot toolbox
import numpy as np

import grinder


RDK = rl.Robolink()
robot = RDK.Item('UR5')
world_frame = RDK.Item('UR5 Base')
target = RDK.Item('Home') # Existing target in station
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())


# Beginning of routine

robot.MoveJ(target, blocking=True)

grinder.home_to_tool_stand_portafilter(robot)
grinder.attach_portafilter(robot, RDK, world_frame)
grinder.tool_stand_to_grinder_portafilter(robot)
grinder.grinder_lower_portafilter(robot)
grinder.detach_portafilter(robot, RDK, world_frame)
grinder.grinder_stand_to_tool_stand(robot)
grinder.attach_grinder_tool(robot, RDK, world_frame)
grinder.tool_stand_to_grinder_buttons(robot)
grinder.press_start_stop_grinder(robot)
