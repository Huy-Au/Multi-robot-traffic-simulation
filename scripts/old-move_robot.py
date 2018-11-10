#!/usr/bin/env python
import rospy
import sys
import tf
import random
from math import atan2, pi, pow, radians, sqrt, fmod
from geometry_msgs.msg import Point, Quaternion, Twist
from tf.transformations import euler_from_quaternion


# Global helper variables
robot_id = 0
tf_listener = ""
base_footprint = ""
cmd_vel= ""

init_x, init_y = 0, 0
intersection_x, intersection_y = 0, 0
southnorth = [0.5, -0.25]
eastwest = [0.25, 0.5]
northsouth = [-0.5, 0.25]
westeast = [-0.25, -0.5]

FORWARD_SPEED = 0.5
FORWARD_SPEED_IN_INTERSECTION = 0.25


def get_robot_position():
	while True:
		try:
			(trans, rot) = tf_listener.lookupTransform('/map', base_footprint, rospy.Time(0))
			(roll, pitch, yaw) = euler_from_quaternion(rot)
			# Converts all yaw to positive values in range 0-2pi
			yaw = fmod((2 * pi) + yaw, 2 * pi)
			# TODO: Robots moving in the north to south direction may have yaw that 
			# switches between 2pi and 0 instaneously. Force values between 340-360 to be 0 degrees
			if yaw > (17 * pi/9) and yaw <= 2 * pi:
				yaw = 0
			return (trans, yaw)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

def move(goal_x, goal_y, turn_bias, isturning):
	distance = sys.maxint
	angle, adjangle, adjcounter = 0, 0, 0
	r = rospy.Rate(3)
	cmd_vel_msg = Twist()

	while distance > 0.3:
		adjcounter += 1
		(trans, yaw) = get_robot_position()
		pos_x, pos_y = trans[0], trans[1]
		distance = sqrt(pow(pos_y - goal_y, 2) + pow(pos_x - goal_x, 2))
		target_angle = atan2(goal_y - pos_y, goal_x - pos_x)
		# Forces all angles (radians) to be positive between 0 and 2pi
		target_angle = fmod((2 * pi) + target_angle, 2 * pi)
		if target_angle > (17 * pi/9) and target_angle <= 2 * pi:
			target_angle = 0
		
		# print("X: %f, Y: %f, Yaw: %f" % (pos_x, pos_y, yaw))
		#print("Target x: %f, y: %f" % (goal_x, goal_y))
		#print("My yaw: %f" % yaw)
		#print("Target angle: %f" % target_angle)

		if adjcounter % turn_bias == 0:
			cmd_vel_msg.angular.z = 0
		else:
			adjangle = target_angle - yaw
			# Edge case where east-west robot is turning left
			if target_angle <= 0.05:
				if abs(yaw - (2 * pi)) < abs(yaw - 0):
					adjangle = (2 * pi) - yaw
			# Edge case where north-south robot is turning right
			if yaw <= 0.05:
				if abs(target_angle - (2 * pi)) < abs(target_angle - 0):
					adjangle = target_angle - (2 * pi)
			cmd_vel_msg.angular.z = 2 * adjangle
		#print("Adj angle: %f" % cmd_vel_msg.angular.z)
		if isturning == False:
			cmd_vel_msg.linear.x = FORWARD_SPEED
		else:
			cmd_vel_msg.linear.x = FORWARD_SPEED_IN_INTERSECTION
		cmd_vel.publish(cmd_vel_msg)
		r.sleep()

	# Stop the robot
	cmd_vel_msg.linear.x = 0.0;
	cmd_vel_msg.angular.z = 0.0;
	cmd_vel.publish(cmd_vel_msg);
	print("robot no. %s stopped" % robot_id)

       

def goToIntersection():
	(trans, yaw) = get_robot_position()
	global intersection_x
	global intersection_y
	pos_x, pos_y = trans[0], trans[1]
	if (pos_y <= 0 and pos_y >= -0.5):
		intersection_x = southnorth[0]
		intersection_y = southnorth[1]
	elif (pos_x <= 0.5 and pos_x >= 0):
		intersection_x = eastwest[0]
		intersection_y = eastwest[1]
	elif (pos_y <= 0.5 and pos_y >= 0):
		intersection_x = northsouth[0]
		intersection_y = northsouth[1]
	else:
		intersection_x = westeast[0]
		intersection_y = westeast[1]
	move(intersection_x, intersection_y, 4, False)


def go_left():
	intersectionLeftTurn()
	if abs(init_x) < abs(init_y):
		move(init_y, init_x, 3, False)
	else:
		move(-init_y, -init_x, 3, False)

def intersectionLeftTurn():
    # Values of init_x and init_y just needs to be swapped but keeping the original
	# sign of initial position.
	global intersection_x
	global intersection_y
	sign_x = 1 if intersection_x >= 0 else - 1
	sign_y = 1 if intersection_y >= 0 else - 1
	if abs(intersection_x) > abs(intersection_y):
		move(sign_x * abs(intersection_y), sign_y * 2 * abs(intersection_x), 3, True)
	else:
		move(sign_x * 2 * abs(intersection_y), sign_y * abs(intersection_x), 3, True)

def go_right():
	move_up()
	intersectionRightTurn()
	if (abs(init_x) > abs(init_y)):
		move(init_y, init_x, 3, False)
	else:
		move(-init_y, -init_x, 3, False)

def move_up():
	(trans, rot) = get_robot_position()
	temp_x, temp_y = trans[0], trans[1]
	#South -> North
	if (temp_y <= 0 and temp_y >= -0.5):
		move(0, temp_y, 2, False)
	#East -> West
	elif (temp_x <= 0.5 and temp_x >= 0):
		move(temp_x, 0, 2, False)
	# North -> South
	elif (temp_y <= 0.5 and temp_y >= 0):
		move(0, temp_y, 2, False)
	# West -> East
	else:
		move(temp_x, 0, 2, False)

def intersectionRightTurn():
	global intersection_x
	global intersection_y
	if abs(intersection_x) > abs(intersection_y):
		move(intersection_y, 2 * intersection_x, 3, True)        
	else:
		move(-2 * intersection_y, -intersection_x, 3, True)

def go_straight():
	intersectionStraight()
	# TODO work out better way of determing end goal
	if abs(init_x) < abs(init_y):
		move(init_x, -init_y, 2, False)
	else:
		move(-init_x, init_y, 2, False)

def intersectionStraight():
	global intersection_x
	global intersection_y
	if abs(intersection_x) < abs(intersection_y):
		move(intersection_x, -intersection_x, 2, False)        
	else:
		move(-intersection_x, intersection_y, 2, False)
	

if __name__ == '__main__':
	robot_id = sys.argv[1]

	# Setting up global helper variables
	node_name = "move_robot_" + str(sys.argv[1])
	rospy.init_node(node_name, anonymous=True)
	cmd_vel_name = "tb" + str(sys.argv[1]) + "/cmd_vel"
	cmd_vel = rospy.Publisher(cmd_vel_name, Twist, queue_size=5)
	tf_listener = tf.TransformListener()
 	base_footprint = "tb" + str(sys.argv[1]) + "/base_footprint"
	(trans, rot) = get_robot_position()
	init_x = trans[0]
	init_y = trans[1]

	random_no = random.randint(0,3)
	goToIntersection()
	if random_no == 0:
		print("Going straight")
		go_straight()
	elif random_no == 1:
		print("Going left")
		go_left()
	else:
		print("Going right")
		go_right()
