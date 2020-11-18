# Thesis experiments

This repository containts the files to conduct the final experiments for my thesis.

### Installing




## Run simulation

The run scripts are in the main folder.

The main run scripts:
```
run_metacontroller.sh
```
Can be used to run the self-adaptive systems.
In the script, the configuration can be changed.

configs: the initial configuration for the self-adaptive system.
scenarios: the scenarios that are used to spawn the obstacles. S1: only obstacles in area1. S2: only obstacles in area2. S3: obstacles in area1 and area2.
mods: the modifications that are used. mod0 is the default MROS system. mod1 is MROS with quality models. mod2 is MROS with quality models and switching back.
runs: the amount of runs per scenario and per system.

The repository ahxl_gazebo is used to spawn the supermarket world.
The script run_single_sim.sh is used to do the simulation.

```
run_benchmarks.sh
```
In the script, the configuration can be changed. Very similar to the run_metacontroller.sh script.


Bag files are generated and stored in the bags folder with a unique name.

## Scripts
```
postprocess_experiments_v2.py 
```
This script can be used to load the latest versions of the bagfiles, and store the data in a pickle file. This is done to speed up the plot process. Loading the data from the pickle file is much faster than loading the bagfiles.

```
plot_metrics_v2.py
```
This script can be used to plot the results of the experiment metrics. Using the pickle file that is generated in postprocess_experiments_v2.py .

```
plot_data.py
```
This script can be used to plot some data. Using the pickle file that is generated in postprocess_experiments_v2.py .


