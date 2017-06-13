#! /usr/bin/env python

import rospy
from mpex_interface import MpexClient
from turtlebot_api_client import TurtlebotClient

turtlebot_client = TurtlebotClient('turtlebot')

# define trajectories
trajectories = {}
 
Atraj = {}
Atraj['B'] = [(0.0, 0.0),(-0.54,0.231),(-0.38,0.743),(-0.073,1.306),(-0.008, 2.42)]
Atraj['C'] = [(0,0),(-0.54, 0.231),(-0.48, 0.743),(-0.684,1.110),(-0.969,1.415)]
Atraj['D'] = [(0,0),(-0.54, 0.231),(-0.48, 0.743),(-0.684,1.110),(-0.969,1.415),(-1.247,1.000),(-1.258,0.762),(-2.018,0.038)]
trajectories['A'] = Atraj

Btraj = {}
Btraj['A'] = [(-0.008,2.420),(0.0, 1.854),(-0.073,1.306),(-0.38,0.743),(-0.54,0.231),(0.1,0)]
Btraj['C'] = [(-0.008,2.420),(0.0, 1.854),(-0.016,0.781),(-0.684,1.110),(-0.969,1.415)]
Btraj['D'] = [(-0.008,2.420),(0.0, 1.854),(-0.016,0.781),(-0.684,1.110),(-0.969,1.415),(-1.247,1.000),(-1.258,0.762),(-2.018,0.038)]
trajectories['B'] = Btraj

Ctraj = {}
Ctraj['A'] = [(-0.969,1.415),(-0.684,1.110),(-0.54, 0.231),(0.1,0)]
Ctraj['B'] = [(-0.969,1.415),(-0.684,1.110),(-0.48, 0.743),(-0.073,1.306),(-0.008,2.42)]
Ctraj['D'] = [(-0.969,1.415),(-1.247,1.000),(-1.258,0.762),(-2.018,0.038)]
trajectories['C'] = Ctraj

Dtraj = {}
Dtraj['A'] = [(-2.018,0.038),(-1.268,0.802),(-1.247,1.000),(-0.969,1.215),(-0.684,1.110),(-0.54, 0.231),(0.1,0)]
Dtraj['B'] = [(-2.018,0.038),(-1.268,0.802),(-1.247,1.000),(-0.969,1.415),(-0.684,1.110),(-0.48, 0.743),(-0.073,1.306),(-0.008,2.42)]
Dtraj['C'] = [(-2.018,0.038),(-1.268,0.802),(-1.247,1.000),(-0.969,1.415)]
trajectories['D'] = Dtraj
   
def move(lb, ub, robot, from_name, to_name):
	path = trajectories[from_name][to_name]
	path_x = [point[0] for point in path]
	path_y = [point[1] for point in path]

	turtlebot_client.follow_trajectory(path_x,path_y)


def explore(lb, ub, robot):
	pass


if __name__ == '__main__':
	rospy.init_node('turtle_handler', anonymous=True)
	mpex_client = MpexClient()
	mpex_client.add_listener('MOVE', move, args=['turtle'])
	mpex_client.add_listener('EXPLORE', explore, args=['turtle'])
	mpex_client.run()