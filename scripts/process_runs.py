#!/usr/bin/env python3.7

import pandas as pd
import robot_primitives as rp
import shapely
import utm
import numpy as np
import sys


run_folder = sys.argv[1]
domain_filename = sys.argv[2]

# Load Domain file and generate all domain objects
domain_path = rp.paths.ConstrainedPath.from_file(domain_filename)
domain_utm_coords = np.array([np.array(utm.from_latlon(lat, lon)[:2]) for lat, lon in domain_path.coord_list])
domain_poly = shapely.geometry.Polygon(domain_utm_coords)
domain_area = domain_poly.area

sensor_radius = 4.5

# Initialize results list that will be appended to as runs are processed
results = []

#Iterate through all runs in specified folder (hardcoded range for now)
for run_id in range(2, 22):
	run_type = 'ee' if run_id % 2 > 0 else 'bous'

	run_gps_filename = f"{run_folder}/run_{run_id}_gps_path.json"
	run_wp_filename = f"{run_folder}/run_{run_id}_wp_path.json"

	run_gps_path = rp.paths.ConstrainedPath.from_file(run_gps_filename)
	run_wp_path = rp.paths.ConstrainedPath.from_file(run_wp_filename)

	run_gps_coords = np.array([np.array(pt) for pt in run_gps_path.coord_list])
	run_wp_coords = np.array([np.array(pt) for pt in run_wp_path.coord_list])
	run_wp_times = np.array([t for t in run_wp_path.time])

	utm_coords = np.array([np.array(utm.from_latlon(lat, lon)[:2]) for lat, lon in run_wp_coords])

	transect_data = []
	for i, transect in enumerate(zip(utm_coords, utm_coords[1:])):
		t_length = np.linalg.norm(transect[1] - transect[0])
		t_duration = run_wp_times[i+1] - run_wp_times[i]
		print(f"Transect {i} Length: {t_length}, Duration: {t_duration}, Speed: {t_length/t_duration}")
		transect_data.append((t_length, t_duration, t_length/t_duration))

	gps_utm_coords = np.array([np.array(utm.from_latlon(lat, lon)[:2]) for lat, lon in run_gps_coords])

	gps_line = shapely.geometry.LineString(gps_utm_coords)
	sensor_coverage = gps_line.buffer(sensor_radius)
	bounded_sensor_coverage = sensor_coverage.intersection(domain_poly)

	coverage_rate = bounded_sensor_coverage.area/domain_area
	print(f"Total Sensor Coverage: {sensor_coverage.area}, Domain Area: {domain_area}, Bounded Sensor Coverage: {bounded_sensor_coverage.area}, Coverage Rate: {coverage_rate}")

	run_utm_path = rp.paths.ConstrainedPath(gps_utm_coords)
	run_duration = run_gps_path.time[-1] - run_gps_path.time[0]

	ordered_primary_transect_data = []
	if run_type == 'ee':
		ordered_primary_transect_data = [transect_data[i][2] for i in [7,3,1,5]]
	elif run_type == 'bous':
		ordered_primary_transect_data = [transect_data[i][2] for i in [1,3,5,7]]

	run_data = [run_type, run_utm_path.length, run_duration, coverage_rate, *ordered_primary_transect_data]
	results.append(run_data)

print(results)

df = pd.DataFrame(results, columns=['run_type', 'length', 'duration', 'coverage', 't1_speed', 't2_speed', 't3_speed', 't4_speed'])
print(df.head(20))
df.to_feather('processed_run_results.feather')
print(df.head(20))
