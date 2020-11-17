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

##################
##################
## Import data
##

files = ['test_2020-10-21-11-06-54.bag']

phases = ['monitor','analyse','qa_update','plan','execution']



print("Import datasets")
# files.append('metacontrol__2020-10-06-13-58-41.bag')
df = dict()
os.chdir(dir_bags)
# Import Bag files into pandas
for idx in range(len(files)):
	print(files[idx])
	df = rosbag_pandas.bag_to_dataframe(dir_bags + '/' + files[idx], include=['/phase_times'])
	df.index -= df.index[0]
	df.index = pd.to_timedelta(df.index, unit='s')
	# print(df.columns)

	# print(df['/phase_times/key'].dropna())

	times=dict()
	for phase in phases:
		times[phase] = df[df['/phase_times/key'] == phase]['/phase_times/value'].astype(float)
		# print(df[df['/phase_times/key'] == phase]['/phase_times/value'].astype(float))
		if len(times[phase]) > 0:
			print('average %s time = %s'%(phase,1000*sum(times[phase])/len(times[phase])))
		else:
			print('no %s times found'%phase)

