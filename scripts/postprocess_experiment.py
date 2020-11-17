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

##################
##################
## Configuration
##

# Paths
rospack = rospkg.RosPack()
path = rospack.get_path('thesis_experiments')
dir_bags = path + '/bags/'
dir_figs = path + '/figures/'
dir_results = path + '/results/'

# Experiment name and configs_adapt
xtopics = ['obstacle_density','narrowness']
ytopics = ['safety']
topics = xtopics + ytopics
# topics = ['safety']

# configs_SA = ["dwa_v1_a0_b0","teb_v1_a0_b0"]
configs_SA = ["dwa_v1_a0_b0"]
# configs_benchmark = ["dwa_v1_a0_b0", "dwa_v2_a1_b0"]
# configs_benchmark = ["dwa_v1_a0_b0", "dwa_v1_a1_b0", "dwa_v1_a1_b1", "dwa_v1_a0_b1", "teb_v1_a0_b0", "teb_v1_a1_b0", "teb_v1_a1_b1", "teb_v1_a0_b1", "dwa_v2_a0_b0", "dwa_v2_a1_b0", "dwa_v2_a1_b1", "dwa_v2_a0_b1", "teb_v2_a0_b0", "teb_v2_a1_b0", "teb_v2_a1_b1", "teb_v2_a0_b1"]
configs_benchmark = ["dwa_v1_a0_b0", "dwa_v2_a1_b0"]
systems = ['sa_m0','sa_m1','sa_m2','b']

# Resamplesize and smoothing (rolling)
samplesize = 100
rolling = 1

scenarios = ['S1','S2','S3']
# scenarios = ['S1']

save = True

##################
##################
## Import data
##


files = dict()
os.chdir(dir_bags)
for scenario in scenarios:
	files[scenario] = dict()
	for system in systems:
		files[scenario][system] = dict()
	for config in configs_SA:
		# SA systems
		file = sorted(glob.glob("metacontrol_M0_%s_%s_*.bag"%(config,scenario)))
		files[scenario]['sa_m0'][config] = file[-1]
		file = sorted(glob.glob("metacontrol_M1_%s_%s_*.bag"%(config,scenario)))
		files[scenario]['sa_m1'][config] = file[-1]
		file = sorted(glob.glob("metacontrol_M2_%s_%s_*.bag"%(config,scenario)))
		files[scenario]['sa_m2'][config] = file[-1]
	for config in configs_benchmark:
		# Benchmarks
		file = sorted(glob.glob("benchmark_%s_%s*.bag"%(config,scenario)))
		files[scenario]['b'][config] = file[-1]


print("Import datasets")
def import_data(file,config,system='benchmark'):
	print(file)
	df = rosbag_pandas.bag_to_dataframe(dir_bags + '/' + file, exclude=[''])
	df.index -= df.index[0]
	df.index = pd.to_timedelta(df.index, unit='s')
	# print(df.loc[df['/metrics/start_end/data'] == "end"].index[0])
	df = df[df.index > df.loc[df['/metrics/start_end/data'] == "start"].index[0]]
	df = df[df.index < df.loc[df['/metrics/start_end/data'] == "end"].index[0]]

	df.index -= df.index[0]

	data = pd.DataFrame(df['/diagnostics/status/0/message'])
	data['pos_x'] = df['/amcl_pose/pose/pose/position/x']
	data['pos_y'] = df['/amcl_pose/pose/pose/position/y']
	data['vel_x'] = df['/boxer_velocity_controller/odom/twist/twist/linear/x']
	data['performance'] = df['/metrics/performance/data']

	for topic in topics:
		data[topic] = df[df['/diagnostics/status/0/values/0/key'] == topic]['/diagnostics/status/0/values/0/value'].astype(float)
	data = data.groupby(level=0).mean()
	data = data.resample('%sms'%samplesize).mean()
	data = data.interpolate(method='linear',limit_direction='both')
	data['progress'] = (data['pos_x'].pow(2).add(data['pos_y'].pow(2))).pow(0.5)
		
	# print(data[data['vel_x'] > 0.1].index[0].total_seconds())

	if system == 'sa':
		current_config = df['/current_configuration/data']
		data_we = data.copy()

		y_pos = [data.index[0].total_seconds()]
		w = []
		for t in current_config.dropna().index.total_seconds():
			y_pos.append(t)
			w.append(y_pos[-1]-y_pos[-2])
		w.append(data.index[-1].total_seconds()-y_pos[-1])
		config_adapt = [config]
		idp = 1
		adapt_n = 0
		for c in current_config.dropna():
			config_adapt.append(c)
			if c == 'execution':
				adapt_n+=1
			idp+=1
		idp-=1
		for c in reversed(current_config.dropna()):
			if c == 'execution':
				data_we.drop(data_we.index[[range(int(y_pos[idp]*10),int(y_pos[idp+1]*10))]], inplace=True)
			idp-=1
		return data,data_we,y_pos,w,adapt_n,config_adapt

	return data

# Import Bag files into pandas
# Metacontrol
data = dict()
data_we = dict()
y_pos = dict()
w = dict()
adapt_n = dict()
config_adapt = dict()
for scen in scenarios:
	data[scen] = dict()
	data_we[scen] = dict()
	y_pos[scen] = dict()
	w[scen] = dict()
	adapt_n[scen] = dict()
	config_adapt[scen] = dict()
	for system in systems:
		data[scen][system] = dict()
		data_we[scen][system] = dict()
		y_pos[scen][system] = dict()
		w[scen][system] = dict()
		adapt_n[scen][system] = dict()
		config_adapt[scen][system] = dict()
	for config in configs_SA:
		for system in ['sa_m0','sa_m1','sa_m2']:
			data[scen][system][config],data_we[scen][system][config],y_pos[scen][system][config],w[scen][system][config],adapt_n[scen][system][config],config_adapt[scen][system][config] = import_data(files[scen][system][config],config,'sa')
	for config in configs_benchmark:
		for system in ['b']:
			data[scen][system][config] = import_data(files[scen][system][config],config)

if save:
	print("Save datasets")
	filename = 'experiment_datasets'
	with open(dir_results + filename, 'wb') as file:
	    pickle.dump([data, data_we, systems, configs_SA, configs_benchmark, scenarios, config_adapt, y_pos, w], file)

##################
##################
## Metrics
##
def calc_metrics(data,adapt_n=0):
	# Time to completion
	timetc = (data.index[-1].total_seconds() - 4*adapt_n)
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

	print("%s adaptations"%adapt_n)
	print("Time to completiong 	= %s s"%timetc)
	print("Average safety 		= %s"%round(safety_av,3))
	print("Minimum safety 		= %s"%round(safety_min,3))
	print("Time violate nfr 	= %s s"%time_violate)

	return timetc,performance_av,safety_av,safety_min,time_violate

print("Calculate metrics")


timetc = dict()
performance_av = dict()
safety_av = dict()
safety_min = dict()
time_violate = dict()
for scen in scenarios:
	timetc[scen] = dict()
	performance_av[scen] = dict()
	safety_av[scen] = dict()
	safety_min[scen] = dict()
	time_violate[scen] = dict()
	for system in systems:
		timetc[scen][system] = dict()
		performance_av[scen][system] = dict()
		safety_av[scen][system] = dict()
		safety_min[scen][system] = dict()
		time_violate[scen][system] = dict()
	for config in configs_SA:
		timetc[scen][system][config] = dict()
		performance_av[scen][system][config] = dict()
		safety_av[scen][system][config] = dict()
		safety_min[scen][system][config] = dict()
		time_violate[scen][system][config] = dict()
		for system in ['sa_m0','sa_m1','sa_m2']:
			timetc[scen][system][config],performance_av[scen][system][config],safety_av[scen][system][config],safety_min[scen][system][config],time_violate[scen][system][config] = calc_metrics(data_we[scen][system][config],adapt_n[scen][system][config])
	for config in configs_benchmark:
		system = 'b'
		timetc[scen][system][config] = dict()
		performance_av[scen][system][config] = dict()
		safety_av[scen][system][config] = dict()
		safety_min[scen][system][config] = dict()
		time_violate[scen][system][config] = dict()
		for system in ['b']:
			timetc[scen][system][config],performance_av[scen][system][config],safety_av[scen][system][config],safety_min[scen][system][config],time_violate[scen][system][config] = calc_metrics(data[scen][system][config])

if save:
	print("Save metrics")
	filename = 'experiment_metrics'
	with open(dir_results + filename, 'wb') as file:
	    pickle.dump([configs_SA, configs_benchmark, scenarios, timetc, performance_av, safety_av, safety_min, time_violate], file)

plt.show()