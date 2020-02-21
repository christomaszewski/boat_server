#!/usr/bin/env python3.7

import rosbag
import rospy
import pandas as pd
import datetime
import time
import robot_primitives as rp

import utm
import matplotlib.pyplot as plt
import numpy as np
import glob
import sys
import os
import robot_primitives as rp
import cartopy
import cartopy.crs as ccrs
import cartopy.feature as cfeature
from cartopy.io.img_tiles import OSM, GoogleTiles

plt.ion()

# run_id = int(sys.argv[1])
# run_folder = '.'
# if len(sys.argv) > 2:
# 	run_folder = sys.argv[2]

run_folder = '.'


for run_id in range(23):
	run_gps_filename = f"{run_folder}/run_{run_id}_gps_path.json"
	run_wp_filename = f"{run_folder}/run_{run_id}_wp_path.json"

	run_gps_path = rp.paths.ConstrainedPath.from_file(run_gps_filename)
	run_wp_path = rp.paths.ConstrainedPath.from_file(run_wp_filename)

	run_gps_coords = np.array([np.array(pt) for pt in run_gps_path.coord_list])
	run_wp_coords = np.array([np.array(pt) for pt in run_wp_path.coord_list])

	utm_coords = np.array([np.array(utm.from_latlon(lat, lon)[:2]) for lat, lon in run_gps_coords])

	mean_center = np.mean(utm_coords, axis=0)

	fig = plt.figure(figsize=(12, 12), dpi=100)

	imagery = GoogleTiles()

	data_crs = ccrs.UTM(17)
	ax = fig.add_subplot(1,1,1, projection=imagery.crs)
	extent_buffer = 80.0
	bottom_left = mean_center - extent_buffer
	top_right = mean_center + extent_buffer
	extents = [bottom_left[0], top_right[0], bottom_left[1], top_right[1]]
	ax.set_extent(extents, crs=data_crs)

	ax.add_image(imagery, 18)

	#plt.plot(domain_boundary_coords[:,1], domain_boundary_coords[:,0], '-b', transform=ccrs.PlateCarree())
	print(run_wp_coords)
	plt.plot(run_wp_coords[:,1], run_wp_coords[:,0], '-g', transform=ccrs.PlateCarree())
	plt.plot(utm_coords[:,0], utm_coords[:,1], '-r', transform=ccrs.UTM(17))
	plt.show()
	plt.pause(2)

	plt.savefig(f"run_{run_id}.png")
	plt.close()

#gps_points = [[t.to_sec()-start_time, msg.lat, msg.long] for _, msg, t in bag.read_messages(topics='/gps')]
#utm_coords = np.array([np.array(utm.from_latlon(lat, lon)[:2]) for lat, lon in gps_points])


