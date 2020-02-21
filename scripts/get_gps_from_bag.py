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
import matplotlib.pyplot as plt
import cartopy
import cartopy.crs as ccrs
import cartopy.feature as cfeature
from cartopy.io.img_tiles import OSM, GoogleTiles

gps_log_filename = sys.argv[1]
bag = rosbag.Bag(gps_log_filename)

run_idx_offset = 0
if len(sys.argv) > 2:
	run_idx_offset = int(sys.argv[2])

start_time = bag.get_start_time()
end_time = bag.get_end_time()

topics = bag.get_type_and_topic_info()[1].keys()

print(start_time, end_time, topics)

def unpack(record):
	topic, msg, t = record
	if topic == '/gps':
		return (datetime.datetime.fromtimestamp(t.to_sec()), (topic, msg.lat, msg.long))
	elif topic == '/debug_target':
		return (datetime.datetime.fromtimestamp(t.to_sec()), (topic, msg.x, msg.y))
	elif topic == '/motor_cmd':
		return (datetime.datetime.fromtimestamp(t.to_sec()), (topic, msg.m0, msg.m1))
	else:
		return None

unpacked_bag = [unpack(record) for record in bag.read_messages(topics=['/debug_target','/gps', '/motor_cmd'])]

timestamps, data = zip(*unpacked_bag)

idx = pd.DatetimeIndex(timestamps)

df = pd.DataFrame(data, index=idx, columns=['topic', 'data1', 'data2'])



# Crop data head and tail based on 0.0 motor values
motor_cmds = df.loc[df['topic'] == '/motor_cmd', 'data1':'data2']
non_zero_motor_cmds = motor_cmds[motor_cmds['data1'] != 0.0][motor_cmds['data2'] != 0.0]
print(motor_cmds.head(), non_zero_motor_cmds.head())
print(non_zero_motor_cmds.index[0], non_zero_motor_cmds.index[-1])
df = df.truncate(after=non_zero_motor_cmds.index[-1])


print(len(df.loc[df['topic'] == '/debug_target', :].index), 'num of targets received')
last_timestamp = df.loc[df['topic'] == '/gps', :].index[-1]
# Setting last recorded gps point as last target to make automated parsing work
df.loc[last_timestamp, 'topic'] = '/debug_target'
print(len(df.loc[df['topic'] == '/debug_target', :].index), 'new num of targets after modification')


# Todo: must be a better way to do this. Need to add the last timestamp available to the end of the target times index
target_data = df.loc[df['topic'] == '/debug_target', :]
target_times = target_data.index[::10]

print(target_data.head(), target_data.tail())

num_runs = len(target_times) - 1
print('num runs:', num_runs)

print(motor_cmds[target_times[0]:target_times[-1]].head())

for run_id in range(num_runs):
	motor_cmds = df.loc[df['topic'] == '/motor_cmd', 'data1':'data2'][target_times[run_id]:target_times[run_id+1]]
	non_zero_motor_cmds = motor_cmds[motor_cmds['data1'] != 0.0][motor_cmds['data2'] != 0.0]
	
	gps_data = df.loc[df['topic'] == '/gps', 'data1':'data2'][target_times[run_id]:non_zero_motor_cmds.index[-1]]
	gps_timestamps = [(t-gps_data.index[0]).total_seconds() for t in gps_data.index]
	gps_path = rp.paths.ConstrainedPath(gps_data.values.tolist(), time=gps_timestamps)
	gps_path.save(f"run_{run_idx_offset + run_id}_gps_path.json")

	wp_data = df.loc[df['topic'] == '/debug_target', 'data1':'data2'][target_times[run_id]:target_times[run_id+1]]
	print(len(wp_data), wp_data)
	wp_arrival_times = [(t-wp_data.index[1]).total_seconds() for t in wp_data.index[1:-1]]
	wp_arrival_times.append((non_zero_motor_cmds.index[-1]-wp_data.index[1]).total_seconds())

	wp_gps_points = [gps_data.truncate(before=t).iloc[0].values.tolist() for t in wp_data.index[1:-1]]
	wp_gps_points.append(gps_data.iloc[-1].values.tolist())

	#wp_gps_points = gps_data.iloc[wp_data.index[1:], 'data1':'data2']
	wp_path = rp.paths.ConstrainedPath(wp_gps_points, time=wp_arrival_times)
	wp_path.save(f"run_{run_idx_offset + run_id}_wp_path.json")

print(df.loc[df['topic'] == '/debug_target', 'data1':'data2'].tail())
print(target_times)
exit()


gps_data = df.loc[df['topic'] == '/gps', 'data1':'data2'][target_times[1]:target_times[2]]
gps_timestamps = [(t-gps_data.index[0]).total_seconds() for t in gps_data.index]

gps_points = df.loc[df['topic'] == '/gps', 'data1':'data2'][target_times[1]:target_times[2]].values.tolist()
print(gps_points)
gps_path = rp.paths.ConstrainedPath(gps_points, time=gps_timestamps)

gps_path.save('gps_path.json')


utm_coords = np.array([np.array(utm.from_latlon(lat, lon)[:2]) for lat, lon in gps_points])

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
#plt.plot(planned_path_coords[:,1], planned_path_coords[:,0], '-g', transform=ccrs.PlateCarree())
plt.plot(utm_coords[:,0], utm_coords[:,1], '-r', transform=ccrs.UTM(17))
plt.show()

plt.savefig(f"run_1.png")

#gps_points = [[t.to_sec()-start_time, msg.lat, msg.long] for _, msg, t in bag.read_messages(topics='/gps')]
#utm_coords = np.array([np.array(utm.from_latlon(lat, lon)[:2]) for lat, lon in gps_points])


