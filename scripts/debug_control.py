#!/usr/bin/env python3.7

import numpy as np
import rospy
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.lines import Line2D
import utm

from boat_server import msg
from std_msgs.msg import Float64
import time

#plt.switch_backend('agg')
plt.ion()

class ControllerViz(object):
	def __init__(self):
		self._fig = plt.figure(figsize=(15,12))
		self._ax = self._fig.add_axes([0.05, 0.05, 0.9, 0.9])
		self._ax.set_xlim(-20., 20.)
		self._ax.set_ylim(-10., 20.)
		self._ax.grid()

		self._target = np.array([3.,7.])
		self._src = np.array([-2.,1.])
		self._pos = np.array([-1.,3])
		self._lookahead = np.array([0.,0.])
		self._curr_heading = 0.
		self._desired_heading = 0.

		self._src_circ = Circle(tuple(self._src), radius=0.2, fc='r', zorder=1)
		self._pos_circ = Circle(tuple(self._pos), radius=0.1, fc='b', zorder=3)
		self._lookahead_circ = Circle(tuple(self._lookahead), radius=0.25, fc='cyan', zorder=0)
		self._target_circ = Circle(tuple(self._target), radius=0.2, fc='g', zorder=1)

		self._ideal_line = Line2D(*zip(self._src, self._target), lw=3., alpha=0.5, zorder=-1)
		heading = self._pos + np.array([np.cos(self._curr_heading), np.sin(self._curr_heading)])
		self._heading_line = Line2D(*zip(self._pos, heading), lw=1, color='r', zorder=2) 
		heading = self._pos + np.array([np.cos(self._desired_heading), np.sin(self._desired_heading)])
		self._desired_line = Line2D(*zip(self._pos, heading), lw=1, color='g', zorder=2)

		self._ax.add_artist(self._target_circ)
		self._ax.add_artist(self._src_circ)
		self._ax.add_artist(self._pos_circ)
		self._ax.add_artist(self._lookahead_circ)
		self._ax.add_artist(self._ideal_line)
		self._ax.add_artist(self._heading_line)
		self._ax.add_artist(self._desired_line)

	def update_src(self, new_src):
		self._src = np.array([new_src.x, new_src.y])
		#self._src_circ.center = tuple(self._src)

	def update_target(self, new_target):
		self._target = np.array([new_target.x, new_target.y])
		#self._target_circ.center = tuple(self._target)

	def update_pos(self, new_pos):
		self._pos = np.array([new_pos.x, new_pos.y])
		#self._pos_circ.center = new_pos.x, new_pos.y

	def update_lookahead(self, new_lookahead):
		self._lookahead = np.array([new_lookahead.x, new_lookahead.y])

	def update_heading(self, new_heading):
		self._curr_heading = float(new_heading.data)

	def update_desired(self, new_heading):
		self._desired_heading = float(new_heading.data)

	def show(self):
		plt.show()
		plt.pause(0.1)

	def update(self):
		self._src_circ.center = tuple(self._src)
		self._pos_circ.center = tuple(self._pos)
		self._lookahead_circ.center = tuple(self._lookahead)
		self._target_circ.center = tuple(self._target)
		self._ideal_line.set_data(*zip(self._src, self._target))
		heading = self._pos + np.array([np.cos(self._curr_heading), np.sin(self._curr_heading)])
		self._heading_line.set_data(*zip(self._pos, heading))
		heading = self._pos + np.array([np.cos(self._desired_heading), np.sin(self._desired_heading)])
		self._desired_line.set_data(*zip(self._pos, heading))
		




rospy.init_node('control_viz', anonymous=True)
viz = ControllerViz()
rospy.Subscriber('debug_pos', msg.debug_pos, viz.update_pos)
rospy.Subscriber('debug_src', msg.debug_pos, viz.update_src)
rospy.Subscriber('debug_target', msg.debug_pos, viz.update_target)
rospy.Subscriber('debug_lookahead', msg.debug_pos, viz.update_lookahead)
rospy.Subscriber('debug_heading', Float64, viz.update_heading)
rospy.Subscriber('debug_desired', Float64, viz.update_desired)

while True:
	viz.show()
	viz.update()
	time.sleep(0.1)



