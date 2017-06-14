#!/usr/bin/env python
import roslib
import rospy

from turtlebot_vicon_control.srv import Trajectory
from std_srvs.srv import Empty as EmptySrv
from std_msgs.msg import Empty, String, Float64, Bool
from geometry_msgs.msg import PoseArray


class TurtlebotClient(object):

    def __init__(self, name):
        self.name = name

    def follow_trajectory(self, trajectory_x, trajectory_y):
        print("Send new trajectory:", trajectory_x, trajectory_y)
        send_traj = rospy.ServiceProxy(self.name+'/send_trajectory', Trajectory)
        response = send_traj(trajectory_x,trajectory_y)
        print(response)
        return response

    def explore(self):
        print("Exploring!")
        send_explore = rospy.ServiceProxy(self.name+'/send_explore', EmptySrv)
        response = send_explore()
        print(response)
        return response


if __name__ == "__main__":

    rospy.init_node('turtlebot_api_client')

    turtlebot = TurtlebotClient('turtlebot')

    rospy.spin()