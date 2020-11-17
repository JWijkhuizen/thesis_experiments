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

# import statistics as stat

from functions_postprocess import *

# Paths
rospack = rospkg.RosPack()
path = rospack.get_path('thesis_experiments')
dir_bags = path + '/bags/'
dir_figs = path + '/figures/'
dir_results = path + '/results/'

# Experiment name and configs
configs = ["dwa_v0_a0_b0", "dwa_v0_a1_b0", "dwa_v0_a1_b1", "dwa_v0_a0_b1", "dwa_v1_a0_b0", "dwa_v1_a1_b0", "dwa_v1_a1_b1", "dwa_v1_a0_b1", "teb_v0_a0_b0", "teb_v0_a1_b0", "teb_v0_a1_b1", "teb_v0_a0_b1", "teb_v1_a0_b0", "teb_v1_a1_b0", "teb_v1_a1_b1", "teb_v1_a0_b1", "dwa_v2_a0_b0", "dwa_v2_a1_b0", "dwa_v2_a1_b1", "dwa_v2_a0_b1", "teb_v2_a0_b0", "teb_v2_a1_b0", "teb_v2_a1_b1", "teb_v2_a0_b1"]
# configs = ["dwa_v0_a0_b0", "dwa_v0_a1_b0", "dwa_v0_a1_b1", "dwa_v0_a0_b1", "dwa_v1_a0_b0", "dwa_v1_a1_b0", "dwa_v1_a1_b1", "dwa_v1_a0_b1", "teb_v0_a0_b0", "teb_v0_a1_b0", "teb_v0_a1_b1", "teb_v0_a0_b1", "teb_v1_a0_b0", "teb_v1_a1_b0", "teb_v1_a1_b1", "teb_v1_a0_b1"]

xtopics = ['obstacle_density','narrowness']
ytopics = ['safety']
topics = xtopics + ytopics
# topics = ['safety']


# Resamplesize and smoothing (rolling)
samplesize = 100
rolling = 1


os.chdir(dir_bags)
files = dict()
for config in configs:
	files[config] = sorted(glob.glob("*%s*.bag"%(config)))
	print(files[config])

# files.append('metacontrol__2020-10-06-13-58-41.bag')
data = dict()
# Import Bag files into pandas
for config in configs:
	data[config] = dict()
	# print(files[config])
	for idx in range(len(files[config])):
		# Import bag
		df = rosbag_pandas.bag_to_dataframe(dir_bags + '/' + files[config][idx], exclude=[''])
		df.index -= df.index[0]
		df.index = pd.to_timedelta(df.index, unit='s')
		# print(df.columns)

		# Determine start and end of experiment and cut the dataset
		try:
			start = df.loc[df['/metrics/start_end/data'] == "start"].index[0].total_seconds()
			end = df.index[-1].total_seconds() - df.loc[df['/metrics/start_end/data'] == "end"].index[0].total_seconds()
		except:
			start = df.index[0].total_seconds()
			end = df.index[1].total_seconds()
		df.drop(df.head(int((start*1000)/samplesize)).index,inplace=True)
		df.drop(df.tail(int((end*1000)/samplesize)).index,inplace=True) # drop last n rows

		# Collect the data of the right topics
		data[config][idx] = pd.DataFrame(df['/diagnostics/status/0/message'])
		data[config][idx]['pos_x'] = df['/amcl_pose/pose/pose/position/x']
		data[config][idx]['pos_y'] = df['/amcl_pose/pose/pose/position/y']
		for topic in topics:
			data[config][idx][topic] = df[df['/diagnostics/status/0/values/0/key'] == topic]['/diagnostics/status/0/values/0/value'].astype(float)
			# if topic == 'safety':
			# 	print(data[topic][data[topic] > 1.5])

		# Resample
		data[config][idx] = data[config][idx].groupby(level=0).mean()
		data[config][idx] = data[config][idx].resample('%sms'%samplesize).mean()
		data[config][idx] = data[config][idx].interpolate(method='linear',limit_direction='both')

		# Determine progress per datapoint
		data[config][idx]['progress'] = (data[config][idx]['pos_x'].pow(2).add(data[config][idx]['pos_y'].pow(2))).pow(0.5)


# # Plot signals against time
# fig, axes = plt.subplots()
# for config in configs:
# 	for topic in ytopics:
# 		axes.plot(data[config][idx][topic].index.total_seconds(), data[config][idx][topic].values, label=config)
# axes.set_ylabel('Signals')
# axes.legend()

idx = 0
safety_average = []
performance = []
for config in configs:
	print(config)
	safety_average.append(data[config][idx]['safety'].mean())
	performance.append(data[config][idx].index[-1].total_seconds())

fig, axes = plt.subplots()
axes.scatter(safety_average,performance)
axes.set_xlabel('Safety average')
axes.set_ylabel('Time to completion [s]')
axes.set_title('Average safety versus time to completion for each configuration')
axes.set_xlim([0,1])

title='safety_vs_performance_v2'
fig.savefig(dir_figs + title + '.png')


# Plot ytopic against progress
fig, axes = plt.subplots()
idx = 0
for config in configs:
	for topic in ytopics:
		axes.plot(data[config][idx]['progress'], data[config][idx][topic], label=config)
axes.legend()
axes.set_ylabel('Safety level')
axes.set_xlabel('Progress [m]')
axes.set_title('Comparison of the safety level of all fixed configuration benchmarks')
fig.tight_layout()

title='safety_fixed_configs_v2'
fig.savefig(dir_figs + title + '.png')

plt.show()