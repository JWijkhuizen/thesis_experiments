#!/usr/bin/env python3.6

import rosbag
import rosbag_pandas
import rospy
import rospkg

import os
import glob

import matplotlib.pyplot as plt
from matplotlib import animation

from openpyxl import Workbook
import pandas as pd
import numpy as np
from matplotlib.widgets import Button
import pickle

from functions_postprocess import *

from collections import defaultdict
from functools import partial

from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures

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
topics = ['obstacle_density','narrowness', 'safety']
xtopics = ['obstacle_density','narrowness']

configs = ['dwa_v1_a0_b0','dwa_v2_a0_b0','dwa_v2_a1_b0']
config_sa = 'dwa_v1_a0_b0'

# Resamplesize and smoothing (rolling)
samplesize = 100 #ms
duration = 10	#s
frames = int(duration*(1000/samplesize))

animate = True
save = True


color = dict()
color['measured']='tab:blue'
color['dwa_v1_a0_b0'] = 'darkgreen'
color['dwa_v1_a1_b0'] = 'green'
color['dwa_v1_a0_b1'] = 'forestgreen'
color['dwa_v1_a1_b1'] = 'limegreen'

color['dwa_v2_a0_b0'] = 'darkslategrey'
color['dwa_v2_a1_b0'] = 'teal'
color['dwa_v2_a0_b1'] = 'cyan'
color['dwa_v2_a1_b1'] = 'deepskyblue'

color['teb_v1_a0_b0'] = 'orangered'
color['teb_v1_a1_b0'] = 'tomato'
color['teb_v1_a0_b1'] = 'chocolate'
color['teb_v1_a1_b1'] = 'sandybrown'

color['teb_v2_a0_b0'] = 'darkorange'
color['teb_v2_a1_b0'] = 'orange'
color['teb_v2_a0_b1'] = 'goldenrod'
color['teb_v2_a1_b1'] = 'yellow'

label = dict()
label['dwa_v1_a0_b0'] = 'dwa1'
label['dwa_v2_a0_b0'] = 'dwa2'
label['dwa_v2_a1_b0'] = 'dwa3'

##################
##################
## Import data
##

model = dict()
for config in configs:
    print('Load model')
    dir_model = rospack.get_path(config)
    pkl_filename = dir_model + "/quality_models/safety.pkl"
    with open(pkl_filename, 'rb') as file:
        model[config] = pickle.load(file)


os.chdir(dir_bags)



files = sorted(glob.glob("video*.bag"))
print(files)

print("Import datasets")
def import_data(file):
	print(file)
	df = rosbag_pandas.bag_to_dataframe(dir_bags + '/' + file, exclude=[''])
	df.index -= df.index[0]
	df.index = pd.to_timedelta(df.index, unit='s')
	# print(df.loc[df['/metrics/start_end/data'] == "end"].index[0])
	df = df[df.index > df.loc[df['/metrics/start_end/data'] == "start"].index[0]]
	# df = df[df.index < df.loc[df['/metrics/start_end/data'] == "end"].index[0]]

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

	print(df.index[0].total_seconds)
	data['configuration'] = config_sa
	time = int(df['/current_configuration/data'].dropna().index.total_seconds().values[1]*(1000/samplesize))
	new_config = df['/current_configuration/data'].dropna()[1]
	data.loc[time:-1,'configuration'] = str(new_config)
	# config_adapt = [config_sa]
	# adapt_n = 0
	# w = []
	# y_pos = [data.index[0].total_seconds()]
	# current_config = df['/current_configuration/data']

	# for t in current_config.dropna().index.total_seconds():
	# 	y_pos.append(t)
	# 	w.append(y_pos[-1]-y_pos[-2])
	# w.append(data.index[-1].total_seconds()-y_pos[-1])
	# idp = 1
	
	# for c in current_config.dropna():
	# 	config_adapt.append(c)
	# 	if c == 'execution':
	# 		adapt_n+=1
	# 	idp+=1
	# idp-=1
	# first = True
	# for c in reversed(current_config.dropna()):
	# 	if first and c == 'execution':
	# 		adapt_n-=1
	# 	if not first:
	# 		if c == 'execution':
	# 			data.drop(data.index[[range(int(y_pos[idp]*10),int(y_pos[idp+1]*10))]], inplace=True)
	# 	first = False
	# 	idp-=1
	return data


# Import Bag files into pandas
data = import_data(files[-1])


