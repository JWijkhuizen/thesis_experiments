#!/usr/bin/env python2.7

import rospy
import rosnode

nodename = '/reasoner'
if __name__ == '__main__':
	nodes = rosnode.get_node_names()
	# print(nodes)
	if nodename in nodes:
		fail = False
	else:
		fail = True

	if fail:
		exitcode=1
	else:
		exitcode=0