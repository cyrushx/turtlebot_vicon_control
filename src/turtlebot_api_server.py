#!/usr/bin/env python
import roslib
import rospy

from turtlebot_vicon_control.srv import Trajectory
from std_msgs.msg import Empty, String, Float64, Bool
from geometry_msgs.msg import TransformStamped, PoseArray, Pose

import math
import tf

class TurtlebotServer(object):
    def __init__(self, name):

        self.name = name

        # Also set up some ROS services
        rospy.Service(self.name+'/send_trajectory', Trajectory,
                      self.send_trajectory_service_callback)

        self.pub_trajectory = rospy.Publisher(
            self.name + '/send_trajectory', PoseArray, queue_size=10)
        
    def send_trajectory_service_callback(self, req):

        print('**Send new trajectory...**')

        traj_x = req.trajectory_x
        traj_y = req.trajectory_y
        traj_len = len(traj_x)

        new_trajectory = []

        for i in range(traj_len):
            pose = Pose()
            pose.position.x = traj_x[i]
            pose.position.y = traj_y[i]

            angle = -math.pi
            quaternion = tf.transformations.quaternion_from_euler(0,0,angle)
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            new_trajectory.insert(len(new_trajectory), pose)

        new_trajectory_message = PoseArray()
        new_trajectory_message.header.stamp = rospy.Time.now()
        new_trajectory_message.poses = new_trajectory

        self.pub_trajectory.publish(new_trajectory_message)

        return "Done"


if __name__ == "__main__":

    rospy.init_node('turblebot_api_server')

    turtlebot_server = TurtlebotServer('turtlebot')

    rospy.spin()