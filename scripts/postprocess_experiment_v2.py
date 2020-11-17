#!/usr/bin/env python

import rosbag
import rosbag_pandas
import rospy
import rospkg

import os
import glob

import matplotlib.pyplot as plt
from openpyxl import Workbook
import pandas as pd
import numpy as np
from matplotlib.widgets import Button
import pickle

from functions_postprocess import *

from collections import defaultdict
from functools import partial
##################
##################
## Configuration
##

# Paths
rospack = rospkg.RosPack()
path = rospack.get_path('final_experiments')
dir_bags = path + '/bags/experiment1/'
dir_figs = path + '/figures/'
dir_results = path + '/results/'

# Experiment name and configs_adapt
topics = ['obstacle_density','narrowness', 'safety']

config_sa = "dwa_v1_a0_b0"
# configs_benchmark = ["dwa_v1_a0_b0", "dwa_v2_a1_b0"]
# configs_benchmark = ["dwa_v1_a0_b0", "dwa_v1_a1_b0", "dwa_v1_a1_b1", "dwa_v1_a0_b1", "teb_v1_a0_b0", "teb_v1_a1_b0", "teb_v1_a1_b1", "teb_v1_a0_b1", "dwa_v2_a0_b0", "dwa_v2_a1_b0", "dwa_v2_a1_b1", "dwa_v2_a0_b1", "teb_v2_a0_b0", "teb_v2_a1_b0", "teb_v2_a1_b1", "teb_v2_a0_b1"]
configs_b = dict()
configs_b['Bf'] = "dwa_v1_a0_b0"
configs_b['Bs'] = "dwa_v2_a1_b0"

systems_sa = ['M0','M1','M2']
# systems_sa = ['M2']

systems_b = ['Bf','Bs']
# systems_b = ['Bf']

scenarios = ['S1','S2','S3']
# scenarios = ['S3']

# runs = [1, 2, 3, 4, 5]
runs = [1, 2, 3, 4]
# runs = [1,2,3,4]
# runs = [4]


systems = systems_b + systems_sa

# Resamplesize and smoothing (rolling)
samplesize = 100 #ms

save = True


def ddd():
	return defaultdict(dd)

def dd():
    return defaultdict(dict)

##################
##################
## Import data
##

os.chdir(dir_bags)
files = defaultdict(lambda: defaultdict(lambda: defaultdict(dict)))
for scenario in scenarios:
	for run in runs:
		for system in systems_sa:
			# SA systems
			file = sorted(glob.glob("metacontrol_%s_%s_%s_r%s*.bag"%(system,config_sa,scenario,run)))
			files[system][scenario][run] = file[-1]
			# print(file[-1])
		for system in systems_b:
			# Benchmarks
			file = sorted(glob.glob("benchmark_%s_%s_r%s*.bag"%(configs_b[system],scenario,run)))
			files[system][scenario][run] = file[-1]
			# print(file[-1])

w_a = []
print("Import datasets")
def import_data(file,system):
	print(file)
	df = rosbag_pandas.bag_to_dataframe(dir_bags + '/' + file, exclude=[''])
	df.index -= df.index[0]
	df.index = pd.to_timedelta(df.index, unit='s')
	# print(df.loc[df['/metrics/start_end/data'] == "end"].index[0])
	df = df[df.index > df.loc[df['/metrics/start_end/data'] == "start"].index[0]]
	df = df[df.index < df.loc[df['/metrics/start_end/data'] == "end"].index[0]]

	df.index -= df.index[0]

	# Add columns to the data dataframe
	data = pd.DataFrame(df['/diagnostics/status/0/message'])
	data['pos_x'] = df['/amcl_pose/pose/pose/position/x']
	data['pos_y'] = df['/amcl_pose/pose/pose/position/y']
	data['vel_x'] = df['/boxer_velocity_controller/odom/twist/twist/linear/x']
	data['performance'] = df['/metrics/performance/data']
	for topic in topics:
		data[topic] = df[df['/diagnostics/status/0/values/0/key'] == topic]['/diagnostics/status/0/values/0/value'].astype(float)

	# Postprocess: resample and interpolate
	data = data.groupby(level=0).mean()
	data = data.resample('%sms'%samplesize).mean()
	data = data.interpolate(method='linear',limit_direction='both')

	# Create a new colums: progress
	data['dis_x'] = data['pos_x'].diff()
	data['dis_y'] = data['pos_y'].diff()
	data['helpercol'] = (data['dis_x']**2 +data['dis_y']**2 )**.5
	data['distance_travelled'] = data['helpercol'].cumsum()
	data['progress'] = (data['pos_x'].pow(2).add(data['pos_y'].pow(2))).pow(0.5)
	# data['goal_distance'] = data['distance_travelled'] - data['distance_travelled'][-1]
	
	# print(data[data['vel_x'] > 0.1].index[0].total_seconds())

	data_we = data.copy()
	config_adapt = [config_sa]
	adapt_n = 0
	w = []
	y_pos = [data.index[0].total_seconds()]
	if system in systems_sa:
		current_config = df['/current_configuration/data']

		
		for t in current_config.dropna().index.total_seconds():
			y_pos.append(t)
			w.append(y_pos[-1]-y_pos[-2])
		w.append(data.index[-1].total_seconds()-y_pos[-1])
		idp = 1
		
		for c in current_config.dropna():
			config_adapt.append(c)
			if c == 'execution':
				adapt_n+=1
			idp+=1
		idp-=1
		# print(adapt_n)
		# print(y_pos)
		# print(data_we['pos_x'])
		first = True
		# print(current_config.dropna())
		for c in reversed(current_config.dropna()):
			if first and c == 'execution':
				adapt_n-=1
			if not first:
				if c == 'execution':
					w_a.append(w[idp])
					print(w[idp])
					# print(int(y_pos[idp]*10))
					# print(int(y_pos[idp+1]*10))
					# print(range(int(y_pos[idp]*10),int(y_pos[idp+1]*10)))
					data_we.drop(data_we.index[[range(int(y_pos[idp]*10),int(y_pos[idp+1]*10))]], inplace=True)
			first = False
			idp-=1
		# print(adapt_n)
	return data,data_we,y_pos,w,adapt_n,config_adapt


# Import Bag files into pandas
# Metacontrol
data = defaultdict(ddd)
data_we = defaultdict(ddd)
y_pos = defaultdict(ddd)
w = defaultdict(ddd)
adapt_n = defaultdict(ddd)
config_adapt = defaultdict(ddd)
failed = []
for scenario in scenarios:
	for run in runs:
		for system in systems:
			# try:
			data[system][scenario][run],data_we[system][scenario][run],y_pos[system][scenario][run],w[system][scenario][run],adapt_n[system][scenario][run],config_adapt[system][scenario][run] = import_data(files[system][scenario][run],system)
			# except Exception as exc:
			# 	print(exc)
			# 	failed.append(files[system][scenario][run])
if len(failed) > 0:
	print("failed:")
	for fail in failed:
		print (fail)
execute_av = sum(w_a)/len(w_a)
print("Average execute time = %s"%execute_av)

if save:
	print("Save datasets")
	filename = 'experiment_datasets_v2'
	with open(dir_results + filename, 'wb') as file:
	    pickle.dump([data, data_we, systems, systems_sa, systems_b, scenarios, runs, config_sa, configs_b, config_adapt, y_pos, w], file)

##################
##################
## Metrics
##
def calc_metrics(data,adapt_n=0):
	# Time to completion
	timetc = (data.index[-1].total_seconds() - execute_av*adapt_n)
	# Average performance
	performance_av = (sum(data['performance'])/len(data['performance']))
	# Average safety
	safety_av = (sum(data['safety'])/len(data['safety']))
	# Minimum safety
	# Time violating nfr
	safety_min = (1.0)
	time_violate = (0)
	timestep = data.index[1].total_seconds() - data.index[0].total_seconds()
	for safety_level in data['safety']:
		if safety_level < 0.6:
			time_violate += timestep
		if safety_level < safety_min:
			safety_min = safety_level

	# print("%s adaptations"%adapt_n)
	# print("Time to completiong 	= %s s"%timetc)
	# print("Average safety 		= %s"%round(safety_av,3))
	# print("Minimum safety 		= %s"%round(safety_min,3))
	# print("Time violate nfr 	= %s s"%time_violate)

	return timetc,performance_av,safety_av,safety_min,time_violate

def calc_av(metric):
	n = len(metric)
	heap = []
	for i in range(len(metric)):
		heap.append(metric[i+1])
	return sum(heap)/len(heap)
	


print("Calculate metrics")

timetc = defaultdict(ddd)
performance_av = defaultdict(ddd)
safety_av = defaultdict(ddd)
safety_min = defaultdict(ddd)
time_violate = defaultdict(ddd)
for scenario in scenarios:
	for system in systems:
		heap = []
		for run in runs:
			timetc[system][scenario][run],performance_av[system][scenario][run],safety_av[system][scenario][run],safety_min[system][scenario][run],time_violate[system][scenario][run] = calc_metrics(data_we[system][scenario][run],adapt_n[system][scenario][run])
			print("%s, %s, r%s, safety_av=%s"%(system,scenario,run,safety_av[system][scenario][run]))
			if safety_av[system][scenario][run] < 0.0 or safety_av[system][scenario][run] > 1.0:
				print("Happens")
				failed.append(files[system][scenario][run])
		timetc[system][scenario][0] = calc_av(timetc[system][scenario])
		performance_av[system][scenario][0] = calc_av(performance_av[system][scenario])
		safety_av[system][scenario][0] = calc_av(safety_av[system][scenario])
		time_violate[system][scenario][0] = calc_av(time_violate[system][scenario])

if len(failed) > 0:
	print("failed:")
	for fail in failed:
		print (fail)

if save:
	print("Save metrics")
	filename = 'experiment_metrics_v2'
	with open(dir_results + filename, 'wb') as file:
	    pickle.dump([systems, systems_sa, systems_b, scenarios, runs, timetc, performance_av, safety_av, safety_min, time_violate], file)

plt.show()