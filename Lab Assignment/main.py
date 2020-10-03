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
# Transform notation {parent}_T_{local_frame}
# RoboDK has trouble with strings, use double quotes, e.g. RDK.RunProgram("Cup Tool Open", True)
# Dragging Python files into RoboDK is not recommended, instead make new empty Python script in RoboDK then paste code in


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
Unused commands:
cup.home_to_tool_stand_cup(robot)
tamper.grinder_stand_to_tamper(robot)

COMMANDS SEQUENCE:
robot.MoveJ(target, blocking=True)
grinder.home_to_tool_stand_portafilter(robot)             # 1.  Move from the home position to entry point for the portafilter on the tool stand
grinder.attach_portafilter(robot, RDK, world_frame)       # 2.  Run program to attach portafilter
grinder.tool_stand_to_grinder_portafilter(robot)          # 3.  Move the portafilter from the tool stand to the coffee grinder
grinder.place_portafilter(robot)                          # 4.  Carefully place the portafilter into position on the coffee grinder
grinder.detach_portafilter(robot, RDK, world_frame)       # 5.  Run program to detach the portafilter and backoff the stand
grinder.stand_to_tool_stand(robot)                        # 6.  Move from the coffee grinder to the entry point for the grinder tool on the tool stand
grinder.attach_grinder_tool(robot, RDK, world_frame)      # 7.  Run program to attach grinder tool
grinder.tool_stand_to_grinder_buttons(robot)              # 8.  Move from the tool stand to the buttons on the side of the coffee grinder
grinder.press_start_stop_grinder(robot, 1)                # 9.  Press the start button, pause [arg 2] seconds, then press the stop button
grinder.approach_grinder_lever(robot)                     # 10. Move into a position where the lever of the coffee grinder can be cranked
grinder.crank_grinder_lever(robot, 70)                    # 11. Crank the lever [arg 2] degrees, then returns to its previous position
grinder.crank_grinder_lever(robot, 70)                    # 12. Crank the lever [arg 2] degrees, then returns to its previous position
grinder.crank_grinder_lever(robot, 70)                    # 13. Crank the lever [arg 2] degrees, then returns to its previous position
grinder.lever_to_tool_stand(robot)                        # 14. Move from the coffee grinder lever position to the grinder tool entry point
grinder.detach_grinder_tool(robot, RDK, world_frame)      # 15. Run program to detach the grinder tool
grinder.tool_stand_to_grinder_portafilter(robot)          # 16. Move from the tool stand to a position ready to approach the portafilter
grinder.grinder_portafilter_reapprch(robot)               # 17. Approach the coffee grinder stand in preparation to connect to the portafilter
grinder.reattach_portafilter(robot, RDK, world_frame)     # 18. Run program to reattach the portafilter tool
tamper.scrape_portafilter(robot)                          # 19. Scraping movement to flatten coffee in the portafilter
tamper.crush_portafilter(robot)                           # 20. Use the tamper disc to crush coffee in the portafilter
coffee_machine.tamper_to_coffee_machine(robot, 2)         # 21. Move to a position where the portafilter can be manually removed and connect to the coffee machine
cup.portafilter_handover_to_cup_tool(robot)               # 22. Move the the stand in preparation to connect to the cup tool
cup.attach_cup_tool(robot, RDK, world_frame, master_tool) # 23. Run program to attach the cup tool
cup.tool_stand_to_cups(robot)                             # 24. Move from the tool stand to the vicinity of the stack of cups
cup.cup_tool_open(robot, RDK, world_frame)                # 25. Run program to open the cup tool
cup.skewer_cup(robot)                                     # 26. Grab a cup from the stack (the 20th cup high)
cup.cup_tool_close(robot, RDK, world_frame)               # 27. Run program to close the cup tool
cup.lift_cup_from_stack(robot)                            # 28. Lift the cup above the stack
coffee_machine.place_cup_in_coffmch(robot)                # 29. Rotate the cup and move to the alcove in the coffee machine
cup.cup_tool_open(robot, RDK, world_frame)                # 30. Run program to open the cup tool
coffee_machine.exit_cup_standoff(robot)                   # 31. Exit out of the stand-off of the coffee machine
cup.cup_tool_close(robot, RDK, world_frame)               # 32. Run program to open the cup tool
coffee_machine.stand_off_to_tool_stand(robot)             # 33. Approach the tool stand entry point for the cup tool
cup.detach_cup_tool(robot, RDK, world_frame)              # 34. Run program to detach the cup tool
coffee_machine.approach_grinder_tool_cup(robot)           # 35. Approach the tool stand entry point for the grinder tool from the cup tool
grinder.attach_grinder_tool(robot, RDK, world_frame)      # 36. Run program to attach the grinder tool
coffee_machine.coffee_switch(robot, 2)                    # 37. Turn the coffee switch on and off with a delay between
coffee_machine.coffee_switch_to_stand(robot)              # 38. Move from the coffee switch to the tool stand entry point for the grinder tool
grinder.detach_grinder_tool(robot, RDK, world_frame)      # 39. Run program to detach the grinder tool
coffee_machine.approach_cup_tool_grinder(robot)           # 40. Approach the tool stand entry point for the cup tool from the grinder tool
cup.attach_cup_tool(robot, RDK, world_frame, master_tool) # 41. Run program to attach the cup tool
coffee_machine.approach_stand_off(robot)                  # 42. Approach the coffee machine stand-off
cup.cup_tool_open(robot, RDK, world_frame)                # 43. Run program to open the cup tool
coffee_machine.skewer_filled_cup(robot)                   # 44. Skewer the filled cup, lift and remove from the stand-off
cup.cup_tool_close(robot, RDK, world_frame)               # 45. Run program to close the cup tool
coffee_machine.serve_cup(robot)                           # 46. Move the cup into a position where it can be taken
'''

# Beginning of the routine

robot.MoveJ(target, blocking=True)
grinder.home_to_tool_stand_portafilter(robot)             # 1.  Move from the home position to entry point for the portafilter on the tool stand
grinder.attach_portafilter(robot, RDK, world_frame)       # 2.  Run program to attach portafilter
grinder.tool_stand_to_grinder_portafilter(robot)          # 3.  Move the portafilter from the tool stand to the coffee grinder
grinder.place_portafilter(robot)                          # 4.  Carefully place the portafilter into position on the coffee grinder
grinder.detach_portafilter(robot, RDK, world_frame)       # 5.  Run program to detach the portafilter and backoff the stand
grinder.stand_to_tool_stand(robot)                        # 6.  Move from the coffee grinder to the entry point for the grinder tool on the tool stand
grinder.attach_grinder_tool(robot, RDK, world_frame)      # 7.  Run program to attach grinder tool
grinder.tool_stand_to_grinder_buttons(robot)              # 8.  Move from the tool stand to the buttons on the side of the coffee grinder
grinder.press_start_stop_grinder(robot, 1)                # 9.  Press the start button, pause [arg 2] seconds, then press the stop button
grinder.approach_grinder_lever(robot)                     # 10. Move into a position where the lever of the coffee grinder can be cranked
grinder.crank_grinder_lever(robot, 70)                    # 11. Crank the lever [arg 2] degrees, then returns to its previous position
grinder.crank_grinder_lever(robot, 70)                    # 12. Crank the lever [arg 2] degrees, then returns to its previous position
grinder.crank_grinder_lever(robot, 70)                    # 13. Crank the lever [arg 2] degrees, then returns to its previous position
grinder.lever_to_tool_stand(robot)                        # 14. Move from the coffee grinder lever position to the grinder tool entry point
grinder.detach_grinder_tool(robot, RDK, world_frame)      # 15. Run program to detach the grinder tool
grinder.tool_stand_to_grinder_portafilter(robot)          # 16. Move from the tool stand to a position ready to approach the portafilter
grinder.grinder_portafilter_reapprch(robot)               # 17. Approach the coffee grinder stand in preparation to connect to the portafilter
grinder.reattach_portafilter(robot, RDK, world_frame)     # 18. Run program to reattach the portafilter tool
tamper.scrape_portafilter(robot)                          # 19. Scraping movement to flatten coffee in the portafilter
tamper.crush_portafilter(robot)                           # 20. Use the tamper disc to crush coffee in the portafilter
coffee_machine.tamper_to_coffee_machine(robot, 2)         # 21. Move to a position where the portafilter can be manually removed and connect to the coffee machine
cup.portafilter_handover_to_cup_tool(robot)               # 22. Move the the stand in preparation to connect to the cup tool
cup.attach_cup_tool(robot, RDK, world_frame, master_tool) # 23. Run program to attach the cup tool
cup.tool_stand_to_cups(robot)                             # 24. Move from the tool stand to the vicinity of the stack of cups
cup.cup_tool_open(robot, RDK, world_frame)                # 25. Run program to open the cup tool
cup.skewer_cup(robot)                                     # 26. Grab a cup from the stack (the 20th cup high)
cup.cup_tool_close(robot, RDK, world_frame)               # 27. Run program to close the cup tool
cup.lift_cup_from_stack(robot)                            # 28. Lift the cup above the stack
coffee_machine.place_cup_in_coffmch(robot)                # 29. Rotate the cup and move to the alcove in the coffee machine
cup.cup_tool_open(robot, RDK, world_frame)                # 30. Run program to open the cup tool
coffee_machine.exit_cup_standoff(robot)                   # 31. Exit out of the stand-off of the coffee machine
cup.cup_tool_close(robot, RDK, world_frame)               # 32. Run program to open the cup tool
coffee_machine.stand_off_to_tool_stand(robot)             # 33. Approach the tool stand entry point for the cup tool
cup.detach_cup_tool(robot, RDK, world_frame)              # 34. Run program to detach the cup tool
coffee_machine.approach_grinder_tool_cup(robot)           # 35. Approach the tool stand entry point for the grinder tool from the cup tool
grinder.attach_grinder_tool(robot, RDK, world_frame)      # 36. Run program to attach the grinder tool
coffee_machine.coffee_switch(robot, 2)                    # 37. Turn the coffee switch on and off with a delay between
coffee_machine.coffee_switch_to_stand(robot)              # 38. Move from the coffee switch to the tool stand entry point for the grinder tool
grinder.detach_grinder_tool(robot, RDK, world_frame)      # 39. Run program to detach the grinder tool
coffee_machine.approach_cup_tool_grinder(robot)           # 40. Approach the tool stand entry point for the cup tool from the grinder tool
cup.attach_cup_tool(robot, RDK, world_frame, master_tool) # 41. Run program to attach the cup tool
coffee_machine.approach_stand_off(robot)                  # 42. Approach the coffee machine stand-off
cup.cup_tool_open(robot, RDK, world_frame)                # 43. Run program to open the cup tool
coffee_machine.skewer_filled_cup(robot)                   # 44. Skewer the filled cup, lift and remove from the stand-off
cup.cup_tool_close(robot, RDK, world_frame)               # 45. Run program to close the cup tool
coffee_machine.serve_cup(robot)                           # 46. Move the cup into a position where it can be taken
