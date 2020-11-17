#!/usr/bin/env python2.7

import rospy
import math
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseActionGoal
import tf2_ros
import time

# Publishers
pub = rospy.Publisher('/metrics/performance', Float64, queue_size=10)


v_max = 1

# Rate and time length
hz = 10
dt = 1.0
n = int(dt * hz)

# Global vars
x_r_min = 0
y_r_min = 0
x_p_min = 0
y_p_min = 0

dis_r = np.zeros(n)
dis_p = np.zeros(n)
dis_pr = np.zeros(n)


# Update the global path
def path_callback(plan):
	global path_poses
	path_poses = plan.poses


def performance_callback(listener):
	global dis_pr, x_r_min, y_r_min, dis_p, x_p_min, y_p_min, path_poses
	try:
		trans = listener.lookup_transform('map', 'base_link', rospy.Time(0))
		# Current positions
		x_r =trans.transform.translation.x
		y_r =trans.transform.translation.y
	except:
		print('Could not get tf')

	try:
		d_path = 10		# Init with large distance
		idpose = 0
		for pose in path_poses:
			d = math.sqrt(pow(pose.pose.position.x-trans.transform.translation.x,2)+pow(pose.pose.position.y-trans.transform.translation.y,2))
			if d < d_path:
				d_path = d
				path_pose = pose
			if idpose > 100:
				# This means that a pose which is closer will no longer be found, so stop searching
				break
			idpose +=1
		x_p = path_pose.pose.position.x
		y_p = path_pose.pose.position.y
		d_p = math.sqrt(pow(x_p_min-x_p,2)+pow(y_p_min-y_p,2))

		d_r = math.sqrt(pow(x_r_min-x_r,2)+pow(y_r_min-y_r,2))
		# Reset if d > 1. This means that the simulation is reset
		if d_r > 10.0:
			dis_p = np.zeros(n)
		else:
			dis_p = dis_p - dis_p[0]
			dis_p = np.append(dis_p,dis_p[-1]+d_p)
			dis_p = np.delete(dis_p, 0)

		# Update previous robot position
		x_r_min = x_r
		y_r_min = y_r
		x_p_min = x_p
		y_p_min = y_p
		
		t_p = dis_p[-1] / v_max
		performance = t_p/dt

		pub.publish(Float64(performance))
	except:
		print('Something went wrong')
	
	# print(str(time.time() - t))

def listener():
	rospy.init_node('observer_performance', anonymous=True)
	# transformations
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)

	rate = rospy.Rate(hz)

	# rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, goal_callback, tfBuffer)
	rospy.Subscriber("/move_base/DWAPlannerROS/global_plan", Path, path_callback)
	rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", Path, path_callback)

	while not rospy.is_shutdown():
		performance_callback(tfBuffer)
		rate.sleep()


if __name__ == '__main__':
	listener()