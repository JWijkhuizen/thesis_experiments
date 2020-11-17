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
ytopics = ['safety','performance']
topics = xtopics + ytopics
# topics = ['safety']




##################
##################
## Import data
##

files = ['metacontrol_M2_dwa_v1_a0_b0_S1_r3_2020-10-25-18-04-21.bag']




print("Import datasets")
# files.append('metacontrol__2020-10-06-13-58-41.bag')
df = dict()
os.chdir(dir_bags)
# Import Bag files into pandas
for idx in range(len(files)):
	print(files[idx])
	df = rosbag_pandas.bag_to_dataframe(dir_bags + '/' + files[idx], exclude=[''])
	df.index -= df.index[0]
	df.index = pd.to_timedelta(df.index, unit='s')
	df['safety'] = df[df['/diagnostics/status/0/values/0/key'] == 'safety']['/diagnostics/status/0/values/0/value'].astype(float)

	time_violation = df[df['safety'] < 1.0].index[0].total_seconds()
	time_adapt = df[df['/log/data'] == 'nfr violation found'].index[0].total_seconds()
	print(time_violation)
	print(time_adapt)
	print(df[df.index.total_seconds() > time_adapt + 3.0]['/log/data'].dropna().head(60))






##################
##################
## Plot log
##

