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
	#
	return rdk.Mat([[5.0783985461E-01, 8.6145149722E-01, 0.0000000000E+00,
	 5.9810000000E+02],
	[-8.6145149722E-01, 5.0783985461E-01, 0.0000000000E+00,
	 4.3100000000E+00],
	[0.0000000000E+00, 0.0000000000E+00, 1.0000000000E+00,
	 2.1258000000E+02],
	[0.0000000000E+00, 0.0000000000E+00, 0.0000000000E+00,
	 1.0000000000E+00]])


def rotate_arm_T():
	# Rotate the end effector 50 degees
	return rdk.TxyzRxyz_2_Pose([0, 0, 0, 0, 0, np.radians(50)])



def grinder_stand_to_tamper(robot):
	servo_positions = [#[2.14, -76.4, -154.12, 230.53, -67.7, 140.0],
	[-8.36, -99.57, -132.34, 231.92, -88.2, 140.0],
	[-8.36, -99.57, -132.34, 231.92, -110.2, 140.0]]


	for pos in servo_positions:
		robot.MoveJ(pos)


def scrape_portafilter(robot):
	#

	# Frames
	tamper_T_scraper = rdk.TxyzRxyz_2_Pose([70, 0, -32, np.radians(-90), 0 , np.radians(-90)])
	portafilter_end_T_tcp = rdk.TxyzRxyz_2_Pose([4.71, 0, 144.76, 0, 0 , 0]).inv()
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
	tamper_T_disc = rdk.TxyzRxyz_2_Pose([-80, 0, -55, np.radians(-90), 0 , np.radians(-90)])
	portafilter_end_T_tcp = rdk.TxyzRxyz_2_Pose([4.71, 0, 144.76, 0, 0 , 0]).inv()
	disc_T_tamper_apprch = rdk.TxyzRxyz_2_Pose([-80, 0, 0, 0, np.radians(7.5) , 0])
	disc_T_crush = rdk.TxyzRxyz_2_Pose([-35, 0, 0, 0, np.radians(7.5) , 0])
	disc_T_tamper_vicinity = rdk.TxyzRxyz_2_Pose([-80, 0, -100, 0, np.radians(7.5) , 0])

	base_T_disc_apprch = base_T_tamperbr()*tamper_T_disc*portafilter_end_T_tcp*disc_T_tamper_apprch*rotate_arm_T()
	base_T_disc = base_T_tamperbr()*tamper_T_disc*portafilter_end_T_tcp*disc_T_crush*rotate_arm_T()
	base_T_disc_vicinity = base_T_tamperbr()*tamper_T_disc*portafilter_end_T_tcp*disc_T_tamper_vicinity*rotate_arm_T()

	robot.MoveJ(base_T_disc_apprch)
	robot.MoveJ(base_T_disc)
	robot.MoveJ(base_T_disc_apprch)
	robot.MoveJ(base_T_disc_vicinity)
