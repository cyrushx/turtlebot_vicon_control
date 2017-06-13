#!/usr/bin/env python

from turtlebot_api_client import TurtlebotClient

turtlebot_client = TurtlebotClient('turtlebot')

# Locations
point_a = [0,0]          # start location
point_b = [-0.078,2.420] # corner next to WAM robot
point_c = [-0.969,1.415] # point in the middle field
point_d = [-1.999,0.111] # corner next to entrance door
point_e = [-2.093,1.416] # corner next to rover

## Starting at a
# path from a to b
path = [[0,0],
        [-0.54,0.231],
        [-0.38,0.743],
        [-0.073,1.306],
        [-0.078,2.42]]

# path from a to c
path = [[0,0],
        [-0.54, 0.231],
        [-0.48, 0.743],
        [-0.684,1.110],
        [-0.969,1.415]]

# path from a to e
path = [[0,0],
        [-0.54, 0.231],
        [-0.48, 0.743],
        [-0.684,1.110],
        [-0.969,1.415],
        [-1.247,1.000],
        [-1.258,0.762],
        [-1.961,0.451],
        [-2.093,1.416]]

# path from a to d
path = [[0,0],
        [-0.54, 0.231],
        [-0.48, 0.743],
        [-0.684,1.110],
        [-0.969,1.415],
        [-1.247,1.000],
        [-1.258,0.762],
        [-2.018,0.038]]

## Starting at b
# path from b to a
path = [[0,0],
        [-0.54,0.231],
        [-0.38,0.743],
        [-0.073,1.306],
        [-0.078,2.42]]

# # path from b to c
# path = [[0,0],
#         [-0.54, 0.231],
#         [-0.48, 0.743],
#         [-0.684,1.110],
#         [-0.969,1.415]]

# # path from b to e
# path = [[0,0],
#         [-0.54, 0.231],
#         [-0.48, 0.743],
#         [-0.684,1.110],
#         [-0.969,1.415],
#         [-1.247,1.000],
#         [-1.258,0.762],
#         [-1.961,0.451],
#         [-2.093,1.416]]

# # path from b to d
# path = [[0,0],
#         [-0.54, 0.231],
#         [-0.48, 0.743],
#         [-0.684,1.110],
#         [-0.969,1.415],
#         [-1.247,1.000],
#         [-1.258,0.762],
#         [-2.018,0.038]]

path_x = [point[0] for point in path]
path_y = [point[1] for point in path]

turtlebot_client.follow_trajectory(path_x,path_y)