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


T_grinder_tool_approach = rdk.TxyzRxyz_2_Pose([-441.77, -214.48, 534.24, 0, np.pi, 0])
T_portafilter_tool_approach = rdk.TxyzRxyz_2_Pose([-399.59, -55.10, 534.02, 0, np.pi, 0])
T_cup_tool_approach = rdk.TxyzRxyz_2_Pose([-358.13, 102.68, 532.67, 0, np.pi, 0])



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
grinder_T_loc = rdk.TxyzRxyz_2_Pose([157.61, 0, -250.45, 0, -np.pi/2, 0])


# Connect to the robot
#rdk.setRunMode(RUNMODE_SIMULATE)
#jnts = robot.Joints()

# Check the connection status and message
#    state, msg = robot.ConnectedState()
#    print(state)
#    print(msg)
#    if state != ROBOTCOM_READY:
#        errors = errors + 'Problems connecting: ' + robot.Name() + ': ' + msg + '\n'
#    else:
#        # move to the joint position in the simulator:
#        robot.MoveJ(jnts, False)

#robot.MoveJ(target, blocking=True)
#robot.MoveJ(T_portafilter_tool_approach, blocking=True)

portafilter_attach.RunCode()
portafilter_attach.WaitFinished()
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())
#T = rdk.TxyzRxyz_2_Pose([4.71, 0, 144.76, 0, np.radians(-7.5), 0])
#R = rdk.TxyzRxyz_2_Pose([32.0, 0, -27.56, 0, np.radians(7.5), 0])

# Not quite right
#robot.MoveJ(base_T_grinder*grinder_T_loc*T*R, blocking=True)



# Inverse?
tcp_T_pf1 = rdk.TxyzRxyz_2_Pose([4.71, 0, 144.76, 0, np.radians(7.5), 0])

tcp_T_pf2 = rdk.TxyzRxyz_2_Pose([32.0, 0, -27.56, 0, np.radians(-7.5), 0])

robot.MoveJ(base_T_grinder*grinder_T_loc*tcp_T_pf1*tcp_T_pf2, blocking=True)

