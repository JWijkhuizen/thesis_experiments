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

# Experiment name and configs_adapt
xtopics = ['obstacle_density','narrowness']
ytopics = ['safety','performance']
topics = xtopics + ytopics
# topics = ['safety']


scatter_safety_vs_performance = True
barplot_bench = False
barplot_timetc = True
barplot_performance = True
barplot_safety = True
barplot_time_violate = True
barplot_safety_performance = False

benchmark_configs = ['dwa_v1_a0_b0','dwa_v2_a1_b0']

##################
##################
## Import
##

filename = 'experiment_metrics'
with open(dir_results + filename) as file:
	configs_sa, configs_b, scenarios, timetc, performance_av, safety_av, safety_min, time_violate = pickle.load(file)

print("configs sa: %s"%configs_sa)
print("configs b: %s"%configs_b)
print("scenarios: %s"%scenarios)
# print("systems: %s"%systems)

##################
##################
## Plot
##

# Av safety vs time
# fig, axes = plt.subplots()
# for scenario in ['S1']:
# 	for config in configs_b:
# 		for system in ['b']:
# 			axes.scatter(safety_av[scenario][system][config],timetc[scenario][system][config],color='blue')
# 	for config in configs_sa:
# 		for system in ['sa_m0','sa_m1']:
# 			axes.scatter(safety_av[scenario][system][config],timetc[scenario][system][config],color='red')
# axes.set_xlabel('Safety average')
# axes.set_ylabel('Time to completion [s]')
# axes.set_title('Average safety versus time to completion for each configuration')
# axes.set_xlim([0,1])

def barplot(metric, ylabel, ylim=False):
	fig, ax = plt.subplots(figsize=fig_normal)
	xlabels = scenarios
	x = np.arange(len(xlabels))  # the label locations
	width = 0.15  # the width of the bars
	idx = 0
	idb = 0
	for system in ['b','b','sa_m0','sa_m1','sa_m2']:
		times = []
		for scenario in scenarios:
			if system == 'b':
				config = benchmark_configs[idb]
				times.append(metric[scenario][system][config])
			else:
				config = 'dwa_v1_a0_b0'
				times.append(metric[scenario][system][config])

		if system == 'b':
			rects = ax.bar(x-2*width+idx*width, times , width, label=label_benchmark[config], color=color_benchmark[config])
			idb+=1
		else:
			rects = ax.bar(x-2*width+idx*width, times , width, label=label_system[system], color=color_system[system])
		autolabel(rects,ax)
		idx+=1

	ax.legend(loc='center', bbox_to_anchor=(0.5, 1.1), ncol=3)
	ax.set_ylabel(ylabel)
	ax.set_xlabel("Scenario")
	# ax.set_title('Time to completion of the SA systems and benchmarks')
	ax.set_xticks(x)
	ax.set_xticklabels(xlabels)
	if ylim:
		ax.set_ylim([0,1])
	ax.yaxis.grid()
	ax.set_axisbelow(True)
	ax.spines['top'].set_visible(False)
	ax.spines['bottom'].set_visible(False)
	ax.spines['right'].set_visible(False)
	ax.spines['left'].set_visible(False)
	plt.subplots_adjust(top=0.85)

	return fig, ax

def barplot_benchmarks(metric, ylabel, scenario, ylim=False):
	fig, ax = plt.subplots(figsize=fig_normal)
	xlabels = configs_b
	x = np.arange(len(xlabels))  # the label locations
	width = 0.9  # the width of the bars
	system = 'b'
	idx = 0
	for config in configs_b:
		value = metric[scenario][system][config]
		rects = ax.bar(x[idx], value , width, color=color_system[system])
		autolabel(rects,ax)
		idx+=1

	# ax.legend(loc='center', bbox_to_anchor=(0.5, 1.1), ncol=3)
	ax.set_ylabel(ylabel)
	ax.set_xlabel("Configuration")
	# ax.set_title('Time to completion of the SA systems and benchmarks')
	ax.set_xticks(x)
	ax.set_xticklabels(xlabels)
	plt.xticks(rotation=90, ha='right')
	if ylim:
		ax.set_ylim([0,1])
	ax.yaxis.grid()
	ax.set_axisbelow(True)
	ax.spines['top'].set_visible(False)
	ax.spines['bottom'].set_visible(False)
	ax.spines['right'].set_visible(False)
	ax.spines['left'].set_visible(False)
	plt.subplots_adjust(top=0.85)

	return fig, ax

if barplot_bench:
	scenario = 'S1'
	ylabel='Average performance'
	fig, ax = barplot_benchmarks(performance_av,ylabel,scenario)
	ylabel='Average safety'
	fig, ax = barplot_benchmarks(safety_av,ylabel,scenario)


# Av safety vs performance av
if scatter_safety_vs_performance:
	for scenario in ['S1','S2','S3']:
		fig, axes = plt.subplots()
		for config in configs_sa:
			for system in ['sa_m0','sa_m1','sa_m2']:
				axes.scatter(safety_av[scenario][system][config],performance_av[scenario][system][config],color=color_system[system],label=label_system[system])
				print(system)
		idx = 1
		for config in configs_b:
			for system in ['b']:
				if idx == 1:
					axes.scatter(safety_av[scenario][system][config],performance_av[scenario][system][config],color=color_system[system],label=label_system[system])
					idx+=1
				else:
					axes.scatter(safety_av[scenario][system][config],performance_av[scenario][system][config],color=color_system[system])
		axes.legend(loc='upper left', bbox_to_anchor=(0, 1.1), ncol=3)
		axes.set_xlabel('Average safety')
		axes.set_ylabel('Average performance')
		# axes.set_title('Average safety versus average performance')
		axes.set_xlim([0,1])
		axes.set_ylim([0,1])
		title = 'scatter_safety_vs_performance_%s'%scenario
		fig.savefig(dir_figs + title + '.png')

# Time to completion barplot
if barplot_timetc:
	ylabel='Time to completion (s)'
	fig, ax = barplot(timetc,ylabel)
	title = 'barplot_timetc'
	fig.savefig(dir_figs + title + '.png')

if barplot_performance:
	ylabel='Average performance level'
	fig, ax = barplot(performance_av,ylabel,True)
	title = 'barplot_performance'
	fig.savefig(dir_figs + title + '.png')


if barplot_safety:
	ylabel='Average safety level'
	fig, ax = barplot(safety_av,ylabel,True)
	title = 'barplot_safety'
	fig.savefig(dir_figs + title + '.png')



if barplot_time_violate:
	ylabel='Violation time (s)'
	fig, ax = barplot(time_violate,ylabel)
	title = 'barplot_time_violate'
	fig.savefig(dir_figs + title + '.png')



if barplot_safety_performance:
	for scenario in scenarios:
		fig, ax = plt.subplots(figsize=fig_normal)
		xlabels = [label_system[system] for system in ['b','sa_m0','sa_m1','sa_m2']]
		x = np.arange(len(xlabels))  # the label locations
		width = 0.2  # the width of the bars
		idx = 0
		for config in configs_sa:
			values1 = []
			values2 = []
			for system in ['b','sa_m0','sa_m1','sa_m2']:
				values1.append(safety_av[scenario][system][config])
				values2.append(performance_av[scenario][system][config])
			rects = ax.bar(x-width/2, values1 , width, label='safety average', color=color_qa['safety'])
			autolabel(rects,ax)
			rects = ax.bar(x+width/2, values2 , width, label='performance average', color=color_qa['performance'])
			autolabel(rects,ax)
			idx+=1
		ax.legend(loc='upper left', bbox_to_anchor=(0, 1.1), ncol=3)
		ax.set_ylabel('Quality attribute level')
		# ax.set_xlabel("System")
		# ax.set_title('Time to completion of the SA systems and benchmarks')
		ax.set_xticks(x)
		ax.set_xticklabels(xlabels)
		ax.yaxis.grid()
		ax.set_axisbelow(True)
		ax.spines['top'].set_visible(False)
		ax.spines['bottom'].set_visible(False)
		ax.spines['right'].set_visible(False)
		ax.spines['left'].set_visible(False)
		title = 'barplot_safety_performance_%s'%scenario
		fig.savefig(dir_figs + title + '.png')


# fig, ax = plt.subplots(figsize=fig_normal)
# ids = [0,1,2,3,10,16]
# xlabels = []
# for i in ids:
# 	xlabels.append(systems[i])
# # xlabels = ['SA\ndwa','SA\nteb','B DWA','B TEB']
# x = np.arange(len(xlabels))  # the label locations
# width = 0.4  # the width of the bars
# rects1 = ax.bar(x-width/2, [safety_av[i] for i in ids], width, label='Safety')
# rects2 = ax.bar(x+width/2, [performance_av[i] for i in ids], width, label='Performance')
# # Labels
# autolabel(rects1,ax)
# autolabel(rects2,ax)
# ax.set_ylabel('Quality attribute')
# ax.set_xticks(x)
# ax.set_xticklabels(xlabels)
# plt.xticks(rotation=45, ha='right')
# # Title
# # ax.set_title('Quality attributes (average) of the SA systems and benchmarks')
# # Grid
# ax.yaxis.grid()
# ax.set_axisbelow(True)
# ax.spines['top'].set_visible(False)
# # ax.spines['bottom'].set_visible(False)
# ax.spines['right'].set_visible(False)
# ax.spines['left'].set_visible(False)
# # Legend
# ax.legend(loc='center', bbox_to_anchor=(0.5, 1.08), ncol=2)
# # Sizes
# ax.set_ylim([0,1])
# plt.subplots_adjust(bottom=0.22,top=0.9)


plt.show()