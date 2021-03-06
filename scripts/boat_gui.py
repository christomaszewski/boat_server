#!/usr/bin/env python3.7
import flask
#from flask import Flask, render_template#, request, session, g, redirect, url_for, abort, render_template, flash

import rospy
import threading
from std_msgs.msg import String
from boat_server import msg

MAPBOX_ACCESS_KEY = 'pk.eyJ1IjoiY2hyaXN0b21hc3pld3NraSIsImEiOiJjanJtN2h1OTAwZ2lnM3ltdDBmZDFjc3FyIn0.2i83Ad3s4mi9DR6ZLG-CFg'

LEFT, RIGHT, FORWARD, REVERSE, WAYPOINT = "left", "right", "forward", "reverse", "waypoint"
AVAILABLE_COMMANDS = {
	'Left': LEFT,
	'Right': RIGHT,
	'Forward': FORWARD,
	'Reverse': REVERSE,
	'Waypoint': WAYPOINT
}

#current_waypoints = [[-71.241844,42.405818],[-71.242497,42.405859],[-71.24200225032979, 42.406372539763396]]
current_waypoints = []
last_gps_point = (42.405818, -71.241844)
last_heading = 90
magnetic_declination = 14. + 25./60.
board_orientation_offset = 90

# Define ROS subscriber callback functions
def imu_callback(imu_data):
	global last_heading
	last_heading = imu_data.x
	rospy.loginfo(f"Received imu data: {imu_data.x}")

def gps_callback(gps_data):
	global last_gps_point
	last_gps_point = (gps_data.lat, gps_data.long)
	rospy.loginfo(f"Received gps data: {gps_data.lat}, {gps_data.long}")

def wp_callback(wp_data):
	global current_waypoints
	current_waypoints = [list(w.coords[::-1]) for w in wp_data.waypoints]

# ROS node init
threading.Thread(target=lambda: rospy.init_node('gui_node', disable_signals=True)).start()
rospy.Subscriber('gps', msg.gps, gps_callback)
rospy.Subscriber('imu', msg.imu, imu_callback)
rospy.Subscriber('waypoint_cmd', msg.waypoint_cmd, wp_callback)

# Setup flask server and define handlers for routes
app = flask.Flask(__name__)

@app.route('/')
@app.route('/index')
def index():
	return flask.render_template('map_gui.html', ACCESS_KEY=MAPBOX_ACCESS_KEY, COMMANDS=AVAILABLE_COMMANDS)

@app.route('/curr_loc')
def curr_loc():
	global last_gps_point
	global last_heading
	coord_str = f"[{last_gps_point[1]},{last_gps_point[0]}]"
	return flask.render_template('curr_loc.json', COORDS=coord_str, HEADING=last_heading)

@app.route('/curr_wp')
def curr_wp():
	global current_waypoints
	return flask.render_template('curr_wp.json', waypoints=current_waypoints)

@app.route('/<cmd>')
def command(cmd=None):
	print(cmd)
	response = f"Moving {cmd}"
	return response, 200, {'Content-Type': 'text/plain'}