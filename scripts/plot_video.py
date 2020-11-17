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

configs = ['dwa_v1_a0_b0','dwa_v2_a0_b0']
config_sa = 'dwa_v1_a0_b0'

# Resamplesize and smoothing (rolling)
samplesize = 100 #ms
duration = 20	#s
frames = int(duration*(1000/samplesize))
print(frames)

animate = True
save = True


color = dict()
color['measured']='tab:blue'
color['dwa_v1_a0_b0'] = 'darkgreen'
color['dwa_v1_a1_b0'] = 'green'
color['dwa_v1_a0_b1'] = 'forestgreen'
color['dwa_v1_a1_b1'] = 'limegreen'

color['dwa_v2_a0_b0'] = 'orangered'
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


	data['configuration'] = config_sa
	time = int(df['/current_configuration/data'].dropna().index.total_seconds().values[1]*(1000/samplesize))
	new_config = df['/current_configuration/data'].dropna()[1]
	data.loc[time:-1,'configuration'] = str(new_config)
	# print(data['configuration'])

	config_adapt = [config_sa]
	adapt_n = 0
	w = []
	y_pos = [data.index[0].total_seconds()]
	current_config = df['/current_configuration/data']

	for t in current_config.dropna().index.total_seconds():
		y_pos.append(t)
		w.append(y_pos[-1]-y_pos[-2])
	w.append(data.index[-1].total_seconds()-y_pos[-1])
	idp = 1
	
	print(df.index[0].total_seconds())
	for c in current_config.dropna():
		config_adapt.append(c)
		if c == 'execution':
			adapt_n+=1
		idp+=1
	idp-=1
	first = True
	for c in reversed(current_config.dropna()):
		if first and c == 'execution':
			adapt_n-=1
		if not first:
			if c == 'execution':
				id1 = int(y_pos[idp]*(1000/samplesize))
				id2 = int(y_pos[idp+1]*(1000/samplesize))
				for topic in topics:
					# idx=1
					data.loc[id1:id2,topic] = data[topic].values[id1-1]
		first = False
		idp-=1
	return data


# Import Bag files into pandas
data = import_data(files[-1])


print("Plot")
# First set up the figure, the axis, and the plot element we want to animate
fig1 = plt.figure(figsize=[6.4,2.8])
ax = plt.axes(xlim=(0, 12), ylim=(0, 1.1))
ax.set_ylabel('Safety level')
ax.set_xlabel("Distance travelled (m)")
ax.set_title("Measured safety level")

ax.axhline(y=0.6,color='grey',linestyle="--")#,label='NFR')
plt.text(12-4, .56, 'safety constraint', transform=ax.get_xaxis_transform(), color='grey')

line, = ax.plot([], [], lw=2, color=color['measured'], label="Measured")
marker, = ax.plot([], [], lw=2, color=color['measured'], marker='o')


config = ax.text(0.05, 1.5, '', transform=ax.transAxes, fontsize=10,
        verticalalignment='top')

# ax.legend(loc=8, ncol=4)

ax.yaxis.grid()
ax.set_axisbelow(True)
ax.spines['top'].set_visible(False)
ax.spines['bottom'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.spines['left'].set_visible(False)
plt.subplots_adjust(bottom=0.18,left=0.09,right=1.05,top=0.84)





fig2 = plt.figure(figsize=[6.4,2.8])
ax = plt.axes(xlim=(0, 12), ylim=(0, 1.1))
ax.set_ylabel('Safety levels')
ax.set_xlabel("Distance travelled (m)")
ax.set_title("Predicted safety levels")

ax.axhline(y=0.6,color='grey',linestyle="--")#,label='NFR')
plt.text(12-4, .56, 'safety constraint', transform=ax.get_xaxis_transform(), color='grey')

line1, = ax.plot([], [], lw=2, color=color[configs[0]], label=label[configs[0]])
line2, = ax.plot([], [], lw=2, color=color[configs[1]], label=label[configs[1]])
# line3, = ax.plot([], [], lw=2, color=color[configs[2]], label=configs[2])

marker1, = ax.plot([], [], lw=2, color=color[configs[0]], marker='o')
marker2, = ax.plot([], [], lw=2, color=color[configs[1]], marker='o')
# marker3, = ax.plot([], [], lw=2, color=color[configs[2]], marker='o')

props1 = dict(boxstyle='round', facecolor=color[configs[0]])
config1 = ax.text(0.05, 1.5, label[configs[0]], transform=ax.transAxes, fontsize=10,
        verticalalignment='top', bbox=props1)
props2 = dict(boxstyle='round', facecolor=color[configs[1]])
config2 = ax.text(0.3, 1.5, label[configs[1]], transform=ax.transAxes, fontsize=10,
        verticalalignment='top', bbox=props2)
# props3 = dict(boxstyle='round', facecolor=color[configs[2]])
# config3 = ax.text(0.55, 1.1, label[configs[2]], transform=ax.transAxes, fontsize=10,
#         verticalalignment='top', bbox=props3)

ax.legend(loc=8, ncol=4)

ax.yaxis.grid()
ax.set_axisbelow(True)
ax.spines['top'].set_visible(False)
ax.spines['bottom'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.spines['left'].set_visible(False)
plt.subplots_adjust(bottom=0.18,left=0.09,right=1.05,top=0.84)


# initialization function: plot the background of each frame
def init_measured():
    line.set_data([], [])
    marker.set_data([], [])
    config.set_text('')
    # config.set_facecolor(color[data['configuration'].values[0]])
    return line, marker, config



# animation function.  This is called sequentially
def animate_measured(i):
    x = data['distance_travelled'].values[0:i]
    # x = data.index[0:i].total_seconds()
    y = data['safety'].values[0:i]
    line.set_data(x, y)

    x = data['distance_travelled'].values[i]
    y = data['safety'].values[i]
    marker.set_data(x, y)
    # print(data['configuration'].values[i])
    config.set_text("Current configuration: " + label[data['configuration'].values[i]])
    # config.set_facecolor(color[data['configuration'].values[i]])


    # if i > 20:
    	
    return line, marker, config

def init_predicted():
    line1.set_data([], [])
    line2.set_data([], [])
    # line3.set_data([], [])
    marker1.set_data([], [])
    marker2.set_data([], [])
    # marker3.set_data([], [])
    config1.set_color('black')
    config2.set_color('black')
    # config3.set_color('black')
    return line1, line2, marker1, marker2, config1, config2

def color_violate(y):
	if y < 0.6:
		color = 'red'
	else:
		color = 'black'
	return color

pf = PolynomialFeatures(degree=5)
def animate_predicted(i):
	if i > 0:
		Xp = pf.fit_transform(data[xtopics].values[0:i])
		# print(Xp)
		x = data['distance_travelled'].values[0:i]

		y1 = model[configs[0]].predict(Xp)
		y2 = model[configs[1]].predict(Xp)
		# y3 = model[configs[2]].predict(Xp)
		# print(y3)

		line1.set_data(x, y1)
		line2.set_data(x, y2)
		# line3.set_data(x, y3)
		marker1.set_data(x[-1], y1[-1])
		config1.set_color(color_violate(y1[-1]))
		marker2.set_data(x[-1], y2[-1])
		config2.set_color(color_violate(y2[-1]))
		# marker3.set_data(x[-1], y3[-1])
		# config3.set_color(color_violate(y3[-1]))
		return line1, line2, marker1, marker2, config1, config2
	else:
		line1.set_data(0, 0)
		line2.set_data(0, 0)
		# line3.set_data(0, 0)
		return line1, line2


os.chdir(dir_figs)
# Figure 2
if animate: anim = animation.FuncAnimation(fig2, animate_predicted, init_func=init_predicted,
                               frames=frames, interval=samplesize, blit=True)
if save: anim.save('predicted_safety.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

# Figure 1

# call the animator.  blit=True means only re-draw the parts that have changed.
if animate: anim = animation.FuncAnimation(fig1, animate_measured, init_func=init_measured,
                               frames=frames, interval=samplesize, blit=True)
if save: anim.save('measured_safety.mp4', fps=30, extra_args=['-vcodec', 'libx264'])



print("Start time = %ss"%data.index[0].total_seconds())
print("End time   = %ss"%data.index[-1].total_seconds())

plt.show()