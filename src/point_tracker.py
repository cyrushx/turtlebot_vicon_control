#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Pose, PoseArray, Twist, TransformStamped

import tf
import math

#class for a generic data holder
class Data:
	def __init__(self): pass # empty constructor

# object to hold the global state of the system
D = Data()


def init():
	""" initialize everything """
	global D

	# init node
	rospy.init_node("pt")

	# initialize subscribers
	rospy.Subscriber("vicon/turtle_pad/turtle_pad", TransformStamped, vicon_pose_callback)
	rospy.Subscriber("turtlebot/send_trajectory", PoseArray, new_trajectory_callback)

	# initialize tf listener
	D.listener = tf.TransformListener()

	# initialize publishers
	D.vel_pub = rospy.Publisher("mobile_base/commands/velocity", Twist, queue_size = 10)
	# D.getNextPoint_pub = rospy.Publisher("get_next_point", Bool, queue_size = 10)

	# initialize global variables
	D.x_vicon         = 0.0
	D.y_vicon         = 0.0
	D.theta_vicon     = -math.pi

	D.trajectory = []

	# D.trajectory = [[-0.48,0.231,-4.79],
	# 				[-0.38,0.743,-4.707],
	# 				[0.033,1.306,-4.75],
	# 				[-0.078,2.42,-4.56]]

	# D.trajectory = [[-0.48,0.231,-4.79],
	# 				[-0.38,0.743,-4.707],
	# 				[-0.843,1.321,-3.257],
	# 				[-1.227,1.000,-1.838],
	# 				[-1.238,0.762,-1.6],
	# 				[-1.941,0.451,-3.703],
	# 				[-2.073,1.416,-4.729]]

	# D.next_point = D.trajectory.pop(0)

	D.x_desired     = D.x_vicon
	D.y_desired     = D.y_vicon
	D.theta_desired = D.theta_vicon

	# initialize point tracking control gains
	D.k_rho       = 0.5 		# needs to be less than k_alpha and (+) 
	D.k_alpha     = 1.4			# needs to be greater than k_rho and (+) 
	D.k_beta      = -0.2		# needs to be (-)

	# tracking accuracies
	D.x_thresh = 0.1				# [m]
	D.y_thresh = 0.1				# [m]
	D.theta_thresh = 0.16/2			# approx 5 degrees [rad]

	# control loop rate
	D.rate = 10 					# [Hz]

	# threshold for arriving at node and getting next point
	D.arrival_thresh = 0.15			# [m]

	# maximum v and w
	D.L     = 0.177

	D.v_max = 0.2
	D.w_max = D.v_max/D.L
	D.v_min = 0.0
	D.w_mult = 1.4
	D.w_min = D.v_min/D.L*D.w_mult
	D.backwards_extra_percentage = 0.2	# need to increase velocity when moving backwards


def new_trajectory_callback(data):
	"""callback function when received a desired_pose. Stores 
	   desired position """
	global D

	new_trajectory = data.poses
	new_trajectory.append(new_trajectory[-1])
	D.trajectory = []

	for pose in new_trajectory:
		x_new = pose.position.x
		y_new = pose.position.y
		quaternion = (pose.orientation.x,
		pose.orientation.y,
		pose.orientation.z,
		pose.orientation.w) 
		euler = tf.transformations.euler_from_quaternion(quaternion)
		theta_new = norm_angle(euler[2])

		D.trajectory.append((x_new,y_new,theta_new))

	print("Point tracker get new trajectory", D.trajectory)



def vicon_pose_callback(data):
	"""callback function when received a desired_pose. Stores 
	   desired position """
	global D

	D.x_vicon = data.transform.translation.x
	D.y_vicon = data.transform.translation.y

	quaternion = (data.transform.rotation.x,
	data.transform.rotation.y,
	data.transform.rotation.z,
	data.transform.rotation.w) 
	euler = tf.transformations.euler_from_quaternion(quaternion)
	# store angle as between -pi and pi
	D.theta_vicon = norm_angle(euler[2]) + 0.838 - math.pi

	# print "Update coordinate:", D.x_vicon, D.y_vicon, D.theta_vicon
	

def fly_to_set_point():
	"""implement point tracking algorithm"""
	global D

	# calculate change in the x, y, and theta
	x_delta     = D.x_desired - D.x_vicon
	y_delta     = D.y_desired - D.y_vicon
	theta_delta = norm_angle(D.theta_desired - D.theta_vicon)

	# determine if the robot is doing in-place rotation
	inPlaceRot = False

	movingBackwards = False

	# don't move if robot is very close to desired
	if (abs(x_delta)<D.x_thresh and abs(y_delta)<D.y_thresh and abs(theta_delta)<D.theta_thresh):
		v = 0
		w = 0
	else:

		# if the robot is close to the point but the angle is different, do in-place rotatoin
		if (abs(x_delta)<D.x_thresh and abs(y_delta)<D.y_thresh and abs(theta_delta)>D.theta_thresh):
			inPlaceRot = True

		# find desired rho, alpha, and beta
		rho   = math.sqrt(x_delta * x_delta + y_delta * y_delta)
		alpha = norm_angle(-D.theta_vicon + math.atan2(y_delta, x_delta))
		beta  = norm_angle(-D.theta_vicon - alpha + D.theta_desired) 	# beta calculation same for both forward and backwards cases

		# find desired forward velocity and angular velocity
		v = D.k_rho * rho

		if (not inPlaceRot):
			w = D.k_alpha * alpha + D.k_beta * beta
		else:
			# print("In place rotation")
			v = 0
			w = -D.k_beta * 2.0 * theta_delta

		# print("Info, alpha, beta", alpha, beta, w)

	# check if we've arrived at the destination
	dist = math.sqrt(x_delta * x_delta + y_delta * y_delta)
	if (dist < D.arrival_thresh and len(D.trajectory) > 0):
		D.next_point = D.trajectory.pop(0)
		print("New point:", D.next_point)

		D.x_desired     = D.next_point[0]
		D.y_desired     = D.next_point[1]
		D.theta_desired = D.next_point[2]

		# D.getNextPoint_pub.publish(True)


	# put max on velocities
	v = min(abs(v), D.v_max) * math.copysign(1,v)
	w = min(abs(w), D.w_max) * math.copysign(1,w) 
	# only put min on forward velocity if not zero. Increase min if moving backwards
	if (v!=0):
		v = max(abs(v), D.v_min*(1+movingBackwards*D.backwards_extra_percentage)) * math.copysign(1,v) 
	# only put min on angular velocity if not zero and doing in place rotation. Increase min if moving backwards
	if (w!=0 and inPlaceRot):
		w = max(abs(w), D.w_min*(1+movingBackwards*D.backwards_extra_percentage)) * math.copysign(1,w) 

	# stop the robot if there is no point to follow
	if len(D.trajectory) == 0:
		v = 0
		w = 0

	# publish normalized Twist message
	vel_msg = Twist()
	vel_msg.linear.x = v
	vel_msg.angular.z = w
	D.vel_pub.publish(vel_msg)

	# print "cmd vel:", vel_msg.linear.x, vel_msg.angular.z

def norm_angle(angle):
	"""normalize an angle to be within -pi to pi"""
	
	while (angle >= math.pi):
		angle -= 2*math.pi
	while (angle < -math.pi):
		angle += 2*math.pi
	
	return angle


if __name__ == "__main__":
	# initialize everything
	init()

	# set control loop rate
	rate = rospy.Rate(D.rate)

	# run until rospy is shutdown
	while not rospy.is_shutdown():
		fly_to_set_point()
		rate.sleep()
