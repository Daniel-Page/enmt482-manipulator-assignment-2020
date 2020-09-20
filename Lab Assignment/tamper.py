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
	return rdk.Mat([[5.0783985461E-01, 8.6145149722E-01, 0.0000000000E+00,
         5.9810000000E+02],
        [-8.6145149722E-01, 5.0783985461E-01, 0.0000000000E+00,
         4.3100000000E+00],
        [0.0000000000E+00, 0.0000000000E+00, 1.0000000000E+00,
         2.1258000000E+02],
        [0.0000000000E+00, 0.0000000000E+00, 0.0000000000E+00,
         1.0000000000E+00]])