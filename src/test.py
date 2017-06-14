#!/usr/bin/env python

from turtlebot_mpex_client import *
from bebop_api_client import Bebop
import time

# turtlebot_client = TurtlebotClient('turtlebot')

# make bebop client object
bebop1 = Bebop('bebop1')

# Locations
point_a = [0,0]          # start location
point_b = [-0.078,2.420] # corner next to WAM robot
point_c = [-0.969,1.415] # point in the middle field
point_d = [-2.018,0.038] # corner next to entrance door
point_e = [-2.093,1.416] # corner next to rover

move(0,10,'a','A','B')
time.sleep(20)

move(0,10,'a','B','D')
time.sleep(30)

# takeoff command - no input
bebop1.takeoff()

time.sleep(5)

bebop1.fly_to(-2.093, 1.416,0.9)

time.sleep(10)

bebop1.turtlebot_land()

time.sleep(20)


# Two landing options:

# land command - no input
# bebop1.land()

# land on turtlebot command - no input
# bebop1.turtlebot_land()

move(0,10,'a','D','A')
time.sleep(30)
