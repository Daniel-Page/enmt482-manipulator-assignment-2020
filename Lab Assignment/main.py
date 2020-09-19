'''
	ENMT482
	Robotics
	Assignment 2
	Coffee Maker Routine
	Daniel Page & Tom Coulson
'''


# In RoboDK:
# Right click on provided scripts and click run on robot for them to work
# Double click 'Disconnect' for it to work or press stop
# Click 'Get robot position' to show actual robot position on the model


import robolink as rl # RoboDK API
import robodk as rdk  # Robot toolbox
import numpy as np

import grinder
import coffee_machine
import tamper

RDK = rl.Robolink()
robot = RDK.Item('UR5')
world_frame = RDK.Item('UR5 Base')
target = RDK.Item('Home') # Existing target in station
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())

'''
Available commands:

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

coffee_machine.coffee_switch(robot)
coffee_machine.hot_water_switch(robot)
coffee_machine.steam_switch(robot)
coffee_machine.power_switch(robot)
'''

# Beginning of the routine

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
