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
# Note that the transforms for UR52019 seem to be more accurate with respect to the 2020 RoboDK model 


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
master_tool = RDK.Item('Master Tool')
portafilter_tool = RDK.Item('Portafilter Tool')


'''
COMMANDS SEQUENCE:


robot.MoveJ(target, blocking=True)


# Grinder functions

grinder.home_to_tool_stand_portafilter(robot)        # 1. Move from the home position to entry point for the portafilter on the tool stand

grinder.attach_portafilter(robot, RDK, world_frame)  # 2. Run program to attach portafilter

grinder.tool_stand_to_grinder_portafilter(robot)     # 3. Move the portafilter from the tool stand to the coffee grinder

grinder.place_portafilter(robot)                     # 4. Carefully place the portafilter into position on the coffee grinder

grinder.detach_portafilter(robot, RDK, world_frame)  # 5. Run program to detach the portafilter

grinder.stand_to_tool_stand(robot)                   # 6. Move from the coffee grinder to the entry point for the grinder tool on the tool stand

grinder.attach_grinder_tool(robot, RDK, world_frame) # 7. Run program to attach grinder tool

grinder.tool_stand_to_grinder_buttons(robot)         # 8. Move from the tool stand to the buttons on the side of the coffee grinder

grinder.press_start_stop_grinder(robot, 3)           # 9. Press the start button, pause [arg 2] seconds, then press the stop button

grinder.approach_grinder_lever(robot)                # 10. Move into a position where the lever of the coffee grinder can be cranked

grinder.crank_grinder_lever(robot, 120)              # 11. Crank the lever [arg 2] degrees, then returns to its previous position

grinder.lever_to_tool_stand(robot)                   # 12. Move from the coffee grinder lever position to the grinder tool entry point

grinder.detach_grinder_tool(robot, RDK, world_frame) # 13. Run program to detach the grinder tool

grinder.tool_stand_to_grinder_portafilter(robot)     # 14. (same as 3) Move from the tool stand to a position ready to approach the portafilter


# Coffee machine functions

coffee_machine.coffee_switch(robot)

coffee_machine.hot_water_switch(robot)

coffee_machine.steam_switch(robot)

coffee_machine.power_switch(robot)


# Cup functions

cup.home_to_tool_stand_cup(robot)                         # Move from home to the tool stand entry point for the cup tool

cup.attach_cup_tool(robot, RDK, world_frame, master_tool) # Run program to attach the cup tool

cup.tool_stand_to_cups(robot)                             # Move from the tool stand to the vicinity of the stack of cups

cup.cup_tool_open(robot, RDK, world_frame)                # Run program to open the cup tool

cup.skewer_cup(robot)                                     # Grab a cup from the stack (the 20th cup high)

cup.cup_tool_close(robot, RDK, world_frame)               # Run program to close the cup tool

cup.lift_cup_from_stack(robot)                            # Lift the cup above the stack

coffee_machine.place_cup_in_coffmch(robot)                # Rotate the cup and move to the alcove in the coffee machine

'''


# Beginning of the routine

robot.MoveJ(target)

cup.home_to_tool_stand_cup(robot)                         # Move from home to the tool stand entry point for the cup tool

cup.attach_cup_tool(robot, RDK, world_frame, master_tool) # Run program to attach the cup tool

cup.tool_stand_to_cups(robot)                             # Move from the tool stand to the vicinity of the stack of cups

cup.cup_tool_open(robot, RDK, world_frame)                # Run program to open the cup tool

cup.skewer_cup(robot)                                     # Grab a cup from the stack (the 20th cup high)

cup.cup_tool_close(robot, RDK, world_frame)               # Run program to close the cup tool

cup.lift_cup_from_stack(robot)                            # Lift the cup above the stack

coffee_machine.place_cup_in_coffmch(robot)   