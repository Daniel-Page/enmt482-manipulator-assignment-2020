'''
	ENCE464
	Embedded Software and Advanced Computing
	Assignment 2

	Voxel Grid Visualisation
	
	Authors: Daniel Page & George Thiele
'''

import matplotlib.pyplot as plt
import math
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


# Axes vectors
ax.quiver(0, 0, 0, 0.6, 0, 0, arrow_length_ratio=0.5, linewidths=2.5, color='red')
ax.quiver(0, 0, 0, 0, 0.6, 0, arrow_length_ratio=0.5, linewidths=2.5, color='green')
ax.quiver(0, 0, 0, 0, 0, 0.6, arrow_length_ratio=0.5, linewidths=2.5, color='blue')

ax.set_xlim([-1,1])
ax.set_ylim([-1,1])
ax.set_zlim([-1,1])

plt.show()
