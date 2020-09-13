import robolink as rl    # RoboDK API
import robodk as rdk     # Robot toolbox
import numpy as np

RDK = rl.Robolink()

robot = RDK.Item('UR5')
world_frame = RDK.Item('UR5 Base')
target = RDK.Item('Home')   # existing target in station
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())

# Use the RDK Matrix object from to hold pose (its an HT)
base_T_grinder = np.matrix([[-0.73289583, -0.68034087, 0,  484.51],
 				           [ 0.68034087, -0.73289583, 0, -426.6 ],
 				           [     0,            0,     1,  318.38],
 				           [     0,            0,     0,    1   ]])

grinder_P_loc = np.matrix([157.61, 0, -250.45]).T

base_P_loc = base_T_grinder*np.concatenate((grinder_P_loc, np.matrix([1])))
base_P_loc = base_P_loc[0:-1]

base_P_loc = np.concatenate((base_P_loc, np.matrix([np.radians(90),np.radians(60),np.radians(0)]).T))

print(base_P_loc)
pose = rdk.TxyzRxyz_2_Pose(base_P_loc)

robot.MoveL(pose, blocking=True)
