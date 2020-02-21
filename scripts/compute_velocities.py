#!/usr/bin/env python3.7

import pandas as pd
import numpy as np
import sys

df = pd.read_feather('processed_run_results.feather')
print(df.head(20))

results = []

for row_idx in range(len(df)-1):
	consecutive_runs = df.iloc[row_idx:row_idx+2]

	bous_run = consecutive_runs.loc[consecutive_runs['run_type'] == 'bous'].squeeze()
	ee_run = consecutive_runs.loc[consecutive_runs['run_type'] == 'ee'].squeeze()

	v_bous_1 = bous_run['t1_speed']
	v_ee_1 = ee_run['t1_speed']
	v_bous_4 = bous_run['t4_speed']
	v_ee_4 = ee_run['t4_speed']

	#print(v_bous_1, v_bous_4, v_ee_1, v_ee_4)

	v_boat_1 = (v_bous_1 + v_ee_1) / 2.
	v_boat_4 = (v_bous_4 + v_ee_4) / 2.
	v_boat_avg = (v_boat_1 + v_boat_4) / 2.
	v_c_1 = (v_bous_1 - v_ee_1) / 2.
	v_c_4 = (v_ee_4 - v_bous_4) / 2.

	bous_transect_v = bous_run['t1_speed':'t4_speed'].values
	v_c_bous = [bous_transect_v[0] - v_boat_avg, v_boat_avg - bous_transect_v[1], bous_transect_v[2] - v_boat_avg, v_boat_avg - bous_transect_v[3]]

	ee_transect_v = ee_run['t1_speed':'t4_speed'].values
	v_c_ee = [v_boat_avg - ee_transect_v[0], v_boat_avg - ee_transect_v[1], ee_transect_v[2] - v_boat_avg, ee_transect_v[3] - v_boat_avg]

	velocities = [v_boat_1, v_boat_4, v_boat_avg, v_c_1, v_c_4, *v_c_bous, *v_c_ee]
	results.append(velocities)

	print(f"Boat Speed (t1 derived): {v_boat_1}, Boat Speed (t4 derived): {v_boat_4}, Current Speed along t1: {v_c_1}, Current Speed along t4: {v_c_4}, {v_ee_4-v_boat_4}, {v_ee_4-v_boat_1}")

v_df = pd.DataFrame(results, columns=['Boat Speed 1', 'Boat Speed 4', 'Avg Boat Speed', 'T1 Current Speed', 'T4 Current Speed',
													'Bous T1 Speed', 'Bous T2 Speed', 'Bous T3 Speed', 'Bous T4 Speed', 'EE T1 Speed',
													'EE T2 Speed', 'EE T3 Speed', 'EE T4 Speed'])

v_df.to_feather('computed_velocities.feather')

print(v_df.head(19))
