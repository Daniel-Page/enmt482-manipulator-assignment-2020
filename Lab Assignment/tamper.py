'''
	ENMT482
	Robotics
	Assignment 2
	Tamper Functions
	Daniel Page & Tom Coulson
'''


import robodk as rdk # Robot toolbox
import numpy as np


def base_T_tamperbr():
	# Tamper-brush origin in the base reference frame
	base_P_tamperbr = np.matrix([598.1, 4.31, 212.58]).T

	# Tamper-bush addditional point
	base_P_adpnt = np.matrix([557.5, 73.18, 157.58]).T

	# The change in coordinates with respect to the base
	dspmt_vector = base_P_adpnt - base_P_tamperbr

	# Determine the angle from the x axis of the displacement vector
	dspmt_xy_vector = [float(dspmt_vector[0]), float(dspmt_vector[1])] # The displacement in terms of x and y

	# Determine the change in angle between the previous vector point to the new additional point
	tamperbr_P_match = np.matrix([-80.00, 0.00, -55.00]).T
	tamperbr_P_match_xy = [float(tamperbr_P_match[0]), float(tamperbr_P_match[1])]

	base_theta = float(np.arctan2(dspmt_xy_vector[1], dspmt_xy_vector[0]))
	tamperbr_theta = float(np.arctan2(tamperbr_P_match[1], tamperbr_P_match[0]))

	theta = base_theta - tamperbr_theta
	
	# Tamper stand reference frame with respect to the robot base reference frame
	return rdk.TxyzRxyz_2_Pose([base_P_tamperbr[0], base_P_tamperbr[1], base_P_tamperbr[2], 0, 0, theta])


def rotate_arm_T():
	# Rotate the end effector 50 degees
	return rdk.TxyzRxyz_2_Pose([0, 0, 0, 0, 0, np.radians(50)])


def grinder_stand_to_tamper(robot):
	# Move to the tamper stand

	# Intermediate positions
	servo_positions = [[-8.36, -99.57, -132.34, 231.92, -88.2, 140.0],
	[-8.36, -99.57, -132.34, 231.92, -110.2, 140.0]]

	for pos in servo_positions:
		robot.MoveJ(pos)


def scrape_portafilter(robot):
	# Scraping movement to flatten coffee in the portafilter

	# Frames
	tamper_T_scraper = rdk.TxyzRxyz_2_Pose([70, 0, -32, np.radians(-90), 0 , np.radians(-90)])
	portafilter_end_T_tcp = rdk.TxyzRxyz_2_Pose([4.71, 0, 144.76, 0, 0 , 0]).invH()
	scraper_T_apprch = rdk.TxyzRxyz_2_Pose([-40, 0, -50, 0, np.radians(7.5) , 0])
	scraper_T_exit = rdk.TxyzRxyz_2_Pose([-40, 0, 100, 0, np.radians(7.5) , 0])
	scraper_T_tamper_lower = rdk.TxyzRxyz_2_Pose([-80, 0, -80, 0, np.radians(7.5) , 0])
	scraper_T_tamper_common = rdk.TxyzRxyz_2_Pose([-80, -110, -50, 0, np.radians(7.5) , 0])

	base_T_tamper_apprch = base_T_tamperbr()*tamper_T_scraper*portafilter_end_T_tcp*scraper_T_apprch*rotate_arm_T()
	base_T_tamper_exit = base_T_tamperbr()*tamper_T_scraper*portafilter_end_T_tcp*scraper_T_exit*rotate_arm_T()
	base_T_tamper_lower = base_T_tamperbr()*tamper_T_scraper*portafilter_end_T_tcp*scraper_T_tamper_lower*rotate_arm_T()
	base_T_tamper_common = base_T_tamperbr()*tamper_T_scraper*portafilter_end_T_tcp*scraper_T_tamper_common*rotate_arm_T()

	robot.MoveJ(base_T_tamper_apprch) 
	robot.MoveJ(base_T_tamper_exit)
	robot.MoveJ(base_T_tamper_lower)
	robot.MoveJ(base_T_tamper_common)


def crush_portafilter(robot):
	# Use the tamper disc to crush coffee in the portafilter

	# Frames
	tamper_T_disc = rdk.TxyzRxyz_2_Pose([-80, 0, -55, np.radians(-90), 0 , np.radians(-90)])
	portafilter_end_T_tcp = rdk.TxyzRxyz_2_Pose([4.71, 0, 144.76, 0, 0 , 0]).invH()
	disc_T_tamper_apprch = rdk.TxyzRxyz_2_Pose([-80, 0, 0, 0, np.radians(7.5) , 0])
	disc_T_crush = rdk.TxyzRxyz_2_Pose([-35, 0, 0, 0, np.radians(7.5) , 0])
	disc_T_tamper_vicinity = rdk.TxyzRxyz_2_Pose([-80, 0, -100, 0, np.radians(7.5) , 0])
	disc_T_tamper_vicinity_1 = rdk.TxyzRxyz_2_Pose([150, 0, -100, 0, np.radians(7.5) , 0])
	
	base_T_disc_apprch = base_T_tamperbr()*tamper_T_disc*portafilter_end_T_tcp*disc_T_tamper_apprch*rotate_arm_T()
	base_T_disc = base_T_tamperbr()*tamper_T_disc*portafilter_end_T_tcp*disc_T_crush*rotate_arm_T()
	base_T_disc_vicinity = base_T_tamperbr()*tamper_T_disc*portafilter_end_T_tcp*disc_T_tamper_vicinity*rotate_arm_T()
	base_T_disc_vicinity_1 = base_T_tamperbr()*tamper_T_disc*portafilter_end_T_tcp*disc_T_tamper_vicinity_1*rotate_arm_T()

	robot.MoveJ(base_T_disc_apprch)
	robot.MoveJ(base_T_disc)
	robot.MoveJ(base_T_disc_apprch)
	robot.MoveJ(base_T_disc_vicinity)
	robot.MoveJ(base_T_disc_vicinity_1)

