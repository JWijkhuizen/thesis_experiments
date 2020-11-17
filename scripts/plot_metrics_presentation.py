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


scatter_safety_vs_performance = False
scatter_safety_vs_timetc = False
scatter_violationtime_vs_timetc = False
barplot_bench = False
barplot_timetc = True
barplot_performance = False
barplot_safety = True
barplot_time_violate = True
barplot_safety_performance = False

save = True

##################
##################
## Import
##

# functions to be able to import defaultdicts:
def ddd():
	return defaultdict(dd)
def dd():
    return defaultdict(dict)


filename = 'experiment_metrics_v2'
with open(dir_results + filename) as file:
	systems, systems_sa, systems_b, scenarios, runs, timetc, performance_av, safety_av, safety_min, time_violate = pickle.load(file)
# print(systems)
# systems = ['Bf', 'Bs', 'M0', 'M1']
# systems_sa = ['M0','M1']
scenarios = ['S2']
# label_system['Bf'] = 'S1'
# label_system['Bs'] = 'S0'
# label_system['M0'] = 'MROS'
# label_system['M1'] = 'MROS_qm'

# scenarios_names = ['Mis']
##################
##################
## Plot
##

def barplot(metric, ylabel, ylim=False):
	fig, ax = plt.subplots(figsize=fig_short)
	xlabels = ['1','2','3']
	x = np.arange(len(xlabels))  # the label locations
	width = 0.2  # the width of the bars
	idx = 0
	for system in systems:
		times = []
		for scenario in scenarios:
			for run in [0]:
				if system  in systems_b:
					times.append(metric[system][scenario][run])
				else:
					times.append(metric[system][scenario][run])

		rects = ax.bar(x-1.5*width+idx*width, times , width, label=label_system[system], color=color_system[system])
		autolabel(rects,ax)
		idx+=1

	ax.legend(loc='center', bbox_to_anchor=(0.5, 1.1), ncol=4)
	ax.set_ylabel(ylabel)
	ax.set_xlabel("Mission")
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


def barplot_average(metric, ylabel, ylim=False):
	fig, ax = plt.subplots(figsize=fig_normal)
	xlabels = ['']
	x = np.arange(len(xlabels))  # the label locations
	width = 0.2  # the width of the bars
	idx = 0
	for system in systems:
		times = []
		for run in [0]:
			values=[]
			for scenario in scenarios:
				values.append(metric[system][scenario][run])
			times.append(sum(values)/len(values))
		rects = ax.bar(x-2*width+idx*width, times , width, label=label_system[system], color=color_system[system])
		autolabel(rects,ax)
		idx+=1

	ax.legend(loc='center', bbox_to_anchor=(0.5, 1.1), ncol=3)
	ax.set_ylabel(ylabel)
	# ax.set_xlabel("Mission")
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
		value = metric[system][scenario][run]
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


# Av safety vs timetc
if scatter_safety_vs_timetc:
	for scenario in ['S1','S2','S3']:
		for run in [0]:
			fig, axes = plt.subplots()
			for system in systems:
				axes.scatter(safety_av[system][scenario][run],timetc[system][scenario][run],color=color_system[system],label=label_system[system])
			axes.legend(loc='center', bbox_to_anchor=(0.5, 1.05), ncol=4)
			axes.set_xlabel('Average safety level')
			axes.set_ylabel('Time to completion (s)')
			# axes.set_title('Average safety versus average performance')
			axes.set_xlim([0,1])
			# axes.set_ylim([0,1])
			title = 'scatter_safety_vs_timetc_%s'%scenario
			if save: fig.savefig(dir_figs + title + '.png')


# Av safety vs timetc
if scatter_violationtime_vs_timetc:
	fig, axes = plt.subplots(figsize=fig_normal)
	for system in systems:
		times_violate = []
		times_tc = []
		for scenario in ['S1','S2','S3']:
			for run in [0]:
				times_violate.append(time_violate[system][scenario][run])
				times_tc.append(timetc[system][scenario][run])
		time_violate_av = sum(times_violate)/len(times_violate)
		time_tc_av = sum(times_tc)/len(times_tc)
		axes.scatter(time_violate_av,time_tc_av,color=color_system[system],label=label_system[system])
	axes.legend(loc='center', bbox_to_anchor=(0.5, 1.05), ncol=4)
	axes.set_xlabel('Violation time (s)')
	axes.set_ylabel('Time to completion (s)')
	# axes.set_title('Average safety versus average performance')
	# axes.set_xlim([0,1])
	# axes.set_ylim([0,1])
	title = 'scatter_timeviolate_vs_timetc_av'
	if save: fig.savefig(dir_figs + title + '.png')


# Time to completion barplot
if barplot_timetc:
	ylabel='Time to completion (s)'
	fig, ax = barplot_average(timetc,ylabel)
	title = 'barplot_timetc_presentation'
	if save: fig.savefig(dir_figs + title + '.png')


if barplot_safety:
	ylabel='Average safety level'
	fig, ax = barplot_average(safety_av,ylabel,True)
	title = 'barplot_safety_presentation'
	if save: fig.savefig(dir_figs + title + '.png')



if barplot_time_violate:
	ylabel='Violation time (s)'
	fig, ax = barplot_average(time_violate,ylabel)
	title = 'barplot_time_violate_presentation'
	if save: fig.savefig(dir_figs + title + '.png')



plt.show()