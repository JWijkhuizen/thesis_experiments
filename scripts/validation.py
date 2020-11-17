#!/usr/bin/env python3.6

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


from sklearn.linear_model import LinearRegression
from sklearn.model_selection import GroupKFold
from sklearn.preprocessing import PolynomialFeatures
from sklearn.metrics import mean_squared_error, explained_variance_score, r2_score

from collections import defaultdict
from functools import partial
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
topics = xtopics+ytopics

configs = ["dwa_v1_a0_b0","teb_v1_a0_b0"]
# configs = ["dwa_v1_a0_b0", "dwa_v1_a1_b0", "dwa_v1_a1_b1", "dwa_v1_a0_b1", "teb_v1_a0_b0", "teb_v1_a1_b0", "teb_v1_a1_b1", "teb_v1_a0_b1", "dwa_v2_a0_b0", "dwa_v2_a1_b0", "dwa_v2_a1_b1", "dwa_v2_a0_b1", "teb_v2_a0_b0", "teb_v2_a1_b0", "teb_v2_a1_b1", "teb_v2_a0_b1"]

samplesize = 100 #ms

plot = True
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
files = dict()
for config in configs:
	# SA systems
	files[config] = sorted(glob.glob("val*%s*.bag"%(config)))[-1]
	print(files[config])


print("Import datasets")
def import_data(file):
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
	X = data[xtopics].values
	y = data[ytopics].values

	return X,y



colors = ['tab:blue','tab:orange']
r2scores = []
mses = []
for ytopic in ytopics:
    # Import Bag files into pandas
    # X, y = import_data(files[config])

    for config in configs:
        # print('Load model')
        X, y = import_data(files[config])
        dir_model = rospack.get_path(config)
        pkl_filename = dir_model + "/quality_models/" + ytopic + ".pkl"
        with open(pkl_filename, 'rb') as file:
            model = pickle.load(file)


        pf = PolynomialFeatures(degree=5)
        Xp = pf.fit_transform(X)

        y1 = model.predict(Xp)
        for i in range(len(y1)):
            y1[i] = min(y1[i],1)
        yr = y
        for i in range(len(yr)):
            yr[i] = min(yr[i],1)

        # Scoring metrics
        print("mean squared error       = %s"%mean_squared_error(yr,y1))
        print("r2 score                 = %s"%r2_score(yr,y1))
        mses.append(mean_squared_error(yr,y1))
        r2scores.append(r2_score(yr,y1))
        if plot:
	        # print(y1)
	        t = np.linspace(0,(len(yr)-1)/10,len(yr))
	        # print(t[2]-t[1])
	        # print(t)
	        fig, ax = plt.subplots(figsize=[6.4,2.6])
	        ax.plot(t, y1, label='Quality model predicted', color=colors[1])#, score = %s'%(round(m1.score(df[idy][xtopics].values,df[idy][ytopic].values),2)))
	        ax.plot(t, yr, label='Quality observer measured', linestyle='--', color=colors[0])
	        ax.legend(loc=0)
	        # ax.set_title('Best safety model and real %s \n trained on run 1, tested on run %s , config = %s \n rmse = %s'%(ytopic,idy,config,round(mean_squared_error(y, y1),5)))
	        ax.set_ylim(0,1.2)
	        ax.set_xlim(0,26)
	        ax.set_ylabel('Safety level')
	        ax.set_xlabel("Time (s)")
	        # ax.set_title("Model (%s) validation \n for config: %s"%(ytopic,config))
	        plt.tight_layout()

	        os.chdir(dir_figs)
	        if save: fig.savefig(dir_figs + 'model_%s_validation_%s'%(ytopic,config) + '.png')
print("r2scores")
print("average 	= %s"%(sum(r2scores)/len(r2scores)))
print("min		= %s"%(min(r2scores)))
print("max		= %s"%(max(r2scores)))
print("mse")
print("average 	= %s"%(sum(mses)/len(mses)))
print("min		= %s"%(min(mses)))
print("max		= %s"%(max(mses)))
plt.show()