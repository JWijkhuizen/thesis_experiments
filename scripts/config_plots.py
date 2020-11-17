#!/usr/bin/env python

########################################
# Figure sizes
fig_wide=[12.8,4.8]
fig_normal=[6.4, 4.8]
fig_short=[6.4,3.2]
fig_short_wide=[12.8,3.2]
fig_small=[3.2,2.4]

#########################################
# Labels
label_system = dict()
label_system['Bf'] = 'Benchmark fast'
label_system['Bs'] = 'Benchmark safe'
label_system['M0'] = 'MROS mod0'
label_system['M1'] = 'MROS mod1'
label_system['M2'] = 'MROS mod2'

label_benchmark = dict()
label_benchmark['dwa_v1_a0_b0'] = 'benchmark fast'
label_benchmark['dwa_v2_a1_b0'] = 'benchmark safe'
label_benchmark['dwa_v2_a0_b0'] = 'benchmark safe2'

#########################################
# Colors
color_config = dict()
color_config['dwa_v1_a0_b0'] = 'darkgreen'
color_config['dwa_v1_a1_b0'] = 'green'
color_config['dwa_v1_a0_b1'] = 'forestgreen'
color_config['dwa_v1_a1_b1'] = 'limegreen'

color_config['dwa_v2_a0_b0'] = 'darkslategrey'
color_config['dwa_v2_a1_b0'] = 'teal'
color_config['dwa_v2_a0_b1'] = 'cyan'
color_config['dwa_v2_a1_b1'] = 'deepskyblue'

color_config['teb_v1_a0_b0'] = 'orangered'
color_config['teb_v1_a1_b0'] = 'tomato'
color_config['teb_v1_a0_b1'] = 'chocolate'
color_config['teb_v1_a1_b1'] = 'sandybrown'

color_config['teb_v2_a0_b0'] = 'darkorange'
color_config['teb_v2_a1_b0'] = 'orange'
color_config['teb_v2_a0_b1'] = 'goldenrod'
color_config['teb_v2_a1_b1'] = 'yellow'

color_system = dict()
color_system['Bf'] = 'royalblue'
color_system['Bs'] = 'cornflowerblue'
color_system['M0'] = 'firebrick'
color_system['M1'] = 'indianred'
color_system['M2'] = 'lightcoral'

color_qa = dict()
color_qa['safety'] = 'tab:green'
color_qa['performance'] = 'tab:blue'

color_benchmark = dict()
color_benchmark['dwa_v1_a0_b0'] = 'royalblue'
color_benchmark['dwa_v2_a1_b0'] = 'cornflowerblue'
color_benchmark['dwa_v2_a0_b0'] = 'lightsteelblue'