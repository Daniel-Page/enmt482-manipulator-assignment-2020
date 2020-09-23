'''
	ENMT482
	Robotics
	Assignment 2
	Coffee Maker Routine
	Daniel Page & Tom Coulson
'''


# Operating a real UR5 from RoboDK:
# Right click on provided scripts and click run on robot for them to work
# Double click 'Disconnect' for it to work or press stop
# Click 'Get robot position' to show actual robot position on the model
# Only this file needs to be added to RoboDK (assuming the modules are in the same directory as the .rdk file)

# Note that TxyzRxyz_2_Pose(...) uses Euler angles


import robolink as rl # RoboDK API
import robodk as rdk  # Robot toolbox
import numpy as np

import grinder
import coffee_machine
import tamper
import cup


RDK = rl.Robolink()
robot = RDK.Item('UR5')
world_frame = RDK.Item('UR5 Base')
target = RDK.Item('Home') # Existing target in station
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())


'''
Commands Sequence:

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

grinder.approach_grinder_lever(robot)
grinder.crank_grinder_lever(robot)
grinder.grinder_lever_to_tool_stand(robot)
grinder.detach_grinder_tool(robot, RDK, world_frame)
grinder.tool_stand_to_grinder_portafilter(robot)




coffee_machine.coffee_switch(robot)
coffee_machine.hot_water_switch(robot)
coffee_machine.steam_switch(robot)
coffee_machine.power_switch(robot)

cup.home_to_tool_stand_cup(robot)
cup.attach_cup_tool(robot, RDK, world_frame)
cup.tool_stand_to_cups(robot)
cup.lower_tool_to_cups(robot)

cup.cup_tool_open(robot, RDK, world_frame)
cup.cup_tool_close(robot, RDK, world_frame)

'''


# Beginning of the routine


cup.cup_tool_open(robot, RDK, world_frame)

cup.tool_stand_to_cups(robot)


cup.lower_tool_to_cups(robot)

