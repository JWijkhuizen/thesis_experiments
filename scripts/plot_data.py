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
from config_plots import *

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

# Experiment name and config_adapt
xtopics = ['obstacle_density','narrowness']
ytopics = ['safety','performance']
topics = xtopics + ytopics
# topics = ['safety']


plot_safety_configs = False
plot_safety_configs_we = False
plot_reconfigs = True
plot_safety_sa_b = True
subplots_safety_sa_b = True

save = True


##################
##################
## Import
##
def ddd():
	return defaultdict(dd)

def dd():
    return defaultdict(dict)


filename = 'experiment_datasets_v2'
with open(dir_results + filename) as file:
	data, data_we, systems, systems_sa, systems_b, scenarios, runs, config_sa, configs_b, config_adapt, y_pos, w = pickle.load(file)


##################
##################
## Plot
##

# Safety versus time, for the SA system with the (re)configurations
if plot_safety_configs:
	for scenario in ['S1']:
		for run in runs:
			for system in systems_sa:
			# for system in ['M2']:
				fig, axes = plt.subplots(figsize=fig_wide)
				idb = 1
				while idb in range(len(y_pos[system][scenario][run])):
					adapt_time = round(y_pos[system][scenario][run][idb],1)
					axes.axvline(x=adapt_time, color='grey',linestyle='--')
					idb+=1
				axes.plot(data[system][scenario][run]['safety'].index.total_seconds(), data[system][scenario][run]['safety'].values, color=color_qa['safety'])
				axes.plot(data[system][scenario][run]['obstacle_density'].index.total_seconds(), data[system][scenario][run]['obstacle_density'].values, color=color_qa['performance'])
				# axes.plot(data_sa[scenario][config]['vel_x'].index.total_seconds(), data_sa[scenario][config]['vel_x'].values)
				for idb in range(len(w[system][scenario][run])):
					if config_adapt[system][scenario][run][idb] != "execution":
						axes.barh([-0.1],w[system][scenario][run][idb],left=y_pos[system][scenario][run][idb],height=0.1,label=config_adapt[system][scenario][run][idb],color=color_config[config_adapt[system][scenario][run][idb]])
				# Legend
				box = axes.get_position()
				axes.set_position([box.x0, box.y0, box.width * 0.8, box.height])
				axes.legend(loc='upper left', bbox_to_anchor=(1, 1))
				# Title and labels
				axes.set_xlabel('Time [s]')
				axes.set_ylabel('Safety level')
				axes.set_title('Safety level and (re-)configurations \n %s'%label_system[system])
				# fig.tight_layout()
				# title = 'safety_level_reconfigurations_%s'%system
				# if save: fig.savefig(dir_figs + title + '.png')

if plot_safety_configs_we:
	for scenario in ['S2']:
		for run in [1]:
			for system in systems_sa:
				fig, axes = plt.subplots(figsize=fig_wide)
				axes.plot(data_we[system][scenario][run]['safety'].index.total_seconds(), data_we[system][scenario][run]['safety'].values, color=color_qa['safety'])
				axes.plot(data_we[system][scenario][run]['performance'].index.total_seconds(), data_we[system][scenario][run]['performance'].values, color=color_qa['performance'])
				# axes.plot(data_sa[scenario][config]['vel_x'].index.total_seconds(), data_sa[scenario][config]['vel_x'].values)
				for idb in range(len(w[system][scenario][run])):
					if config_adapt[system][scenario][run][idb] != "execution":
						axes.barh([-0.1],w[system][scenario][run][idb],left=y_pos[system][scenario][run][idb],height=0.1,label=config_adapt[system][scenario][run][idb],color=color_config[config_adapt[system][scenario][run][idb]])
				# Legend
				box = axes.get_position()
				axes.set_position([box.x0, box.y0, box.width * 0.8, box.height])
				axes.legend(loc='upper left', bbox_to_anchor=(1, 1))
				# Title and labels
				axes.set_xlabel('Time [s]')
				axes.set_ylabel('Safety level')
				axes.set_title('Safety level and (re-)configurations \n %s'%label_system[system])
				# fig.tight_layout()
				title = 'safety_level_reconfigurations_%s'%system
				if save: fig.savefig(dir_figs + title + '.png')


if plot_reconfigs:
	for scenario in scenarios:
		for run in [1]:
			fig, axes = plt.subplots(figsize=fig_short_wide)
			y = 0
			ys = [y]
			for system in systems_sa:
				t = 0
				for idb in range(len(w[system][scenario][run])):
					if config_adapt[system][scenario][run][idb] != "execution":
						axes.barh([y],w[system][scenario][run][idb],left=t,height=0.75,label=config_adapt[system][scenario][run][idb],color=color_config[config_adapt[system][scenario][run][idb]])
						t += w[system][scenario][run][idb]
				y -= 1
				ys.append(y)
			# Legend
			box = axes.get_position()
			# axes.set_position([box.x0, box.y0, box.width * 0.8, box.height])
			axes.legend(loc='upper left', bbox_to_anchor=(1, 1.03))

			axes.set_yticks(ys)
			axes.set_yticklabels(systems_sa)
			print([ys[-1]+1-0.5,ys[0]+0.5])
			axes.set_ylim([ys[-1]+1-0.5,ys[0]+0.5])
			# Title and labels
			axes.set_xlabel('Time (s)')
			axes.set_ylabel('System')
			# axes.set_title('Safety level and (re-)configurations \n %s'%label_system[system])
			# axes.set_title(scenario)
			# fig.tight_layout()
			plt.subplots_adjust(left=0.07,right=0.87,bottom=0.15)
			title = 'reconfigurations_%s'%scenario
			if save: fig.savefig(dir_figs + title + '.png')

# Plot ytopic against distance travelled
if plot_safety_sa_b:
	for scenario in scenarios:
		for run in [1]:
			fig, axes = plt.subplots(figsize=fig_wide)
			for system in systems:
				axes.plot(data[system][scenario][run]['distance_travelled'], data[system][scenario][run]['safety'], label=label_system[system], color=color_system[system])
			for idb in range(len(y_pos[system][scenario][run])):
				# if config_adapt[system][scenario][run][idb] != "execution":
				adapt_time = round(y_pos[system][scenario][run][idb],1)
				print(adapt_time)
				print(data[system][scenario][run]['distance_travelled'].iloc[int(adapt_time*10)])
				axes.axvline(x=data[system][scenario][run]['distance_travelled'].iloc[int(adapt_time*10)])
			axes.set_ylabel('Safety level')
			axes.legend()
			axes.set_xlabel('Travelled distance (m)')
			axes.set_title('Safety versus progress, scenario %s'%scenario)
			fig.tight_layout()
			title = 'safety_progress_%s'%scenario
			if save: fig.savefig(dir_figs + title + '.png')

if subplots_safety_sa_b:
	for scenario in scenarios:
	# for scenario in ['S1']:
		for run in [1]:
			fig, axes = plt.subplots(len(systems_sa),figsize=fig_wide,sharex=True)
			idp = 0
			for system in systems_sa:
				idb = 1
				while idb in range(len(y_pos[system][scenario][run])):
					adapt_time = round(y_pos[system][scenario][run][idb],1)
					axes[idp].axvline(x=data[system][scenario][run]['distance_travelled'].iloc[int(adapt_time*10)], color='grey',linestyle='--')
					idb+=2
				axes[idp].plot(data[system][scenario][run]['distance_travelled'], data[system][scenario][run]['safety'], label=label_system[system], color=color_system[system])
				axes[idp].set_title(label_system[system])

				axes[idp].axhline(y=0.6,color='grey',linestyle=(0, (3, 10, 1, 10)))
				for system in systems_b:
					if idp==2: axes[idp].plot(data[system][scenario][run]['distance_travelled'], data[system][scenario][run]['safety'], label=label_system[system], color=color_system[system])
				axes[idp].set_ylabel('Safety level')
				idp+=1
			idp-=1
			axes[idp].set_xlabel('Distance travelled (m)')
			lines_labels = [ax.get_legend_handles_labels() for ax in fig.axes]
			lines, labels = [sum(lol, []) for lol in zip(*lines_labels)]
			fig.legend(lines, labels, loc='center', bbox_to_anchor=(0.5, 0.93), ncol=5)
			# axes[0].set_title(scenario)
			for idp in [0,1]:
				for system in systems_b:
					axes[idp].plot(data[system][scenario][run]['progress'], data[system][scenario][run]['safety'], color=color_system[system])
			
			# axes.set_title('Safety versus progress, scenario %s'%scenario)
			fig.tight_layout()
			plt.subplots_adjust(top=0.85)
			title = 'safety_progress_%s'%scenario
			if save: fig.savefig(dir_figs + title + '.png')


plt.show()