#!/usr/bin/env python
import rospy
import sys
import tf
from random import randint
from std_msgs.msg import String
from math import atan2, pi, pow, radians, sqrt, fmod, ceil, floor
from geometry_msgs.msg import Point, Quaternion, Twist
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import DeleteModel
from sensor_msgs.msg import LaserScan

class Robot:
	
	def __init__(self, robot_id):
		node_name = "move_robot_" + str(robot_id)
		rospy.init_node(node_name, anonymous=True)


		self.r = rospy.Rate(10)
		self.START_TIME = rospy.get_time()
		"""
		self.correctionAngle = 17 * pi / 9
		"""
		self.correctionAngle = pi / 4

		## WARNING, TRAVEL DISTANCE MUST BE GREATER THAN THE ROBOT FURTHEREST FROM THE INTERSECTION
		## E.G ROBOT WITH CO_ORDINATES (20, 0.25) will not work with TRAVEL DISTANCE 20. AT LEAST ONE INTEGER BIGGER
		## THAT IS, TRAVEL DISTANCE OF 21 required
		self.TOTALDISTANCETRAVELLED = 25
		self.SAFETY_DISTANCE = 1
		self.FORWARD_SPEED = 0.5
		self.FORWARD_SPEED_IN_INTERSECTION = 0.22
		self.MIN_SCAN_ANGLE = -10 * pi/180
		self.MAX_SCAN_ANGLE = 10 * pi/180
		self.MIN_ADJ_ANGLE = 0.1


		self.INCOMING_DIRECTION = ""
		#IS THIS ONE NECESSARY?		
		self.OUTGOING_DIRECTION = ""
		self.TURNING_DIRECTION = ""
		self.RELATIVE_POSITION = ""
		self.wait_for_controller = False
		self.inside_central_intersection_area = False
		self.SAFE_TO_GO_THROUGH_INTERSECTION = False
		self.SAFE_NORTH_SOUTH = False
		self.SAFE_SOUTH_NORTH = False
		self.SAFE_EAST_WEST = False
		self.SAFE_WEST_EAST = False


		self.robot_id = robot_id
		self.tf_listener = tf.TransformListener()
		self.base_footprint = "tb" + str(robot_id) + "/base_footprint"
		self.cmd_vel = rospy.Publisher("tb" + str(robot_id) + "/cmd_vel", Twist, queue_size=1)
		self.robot_relative_position = rospy.Publisher("tb" + str(robot_id) + "_relative_position", String, queue_size = 10)
		self.robot_listener = rospy.Subscriber("tb" + str(robot_id) + "_talker", String, self.listen_callback)
		self.laser_scan_sub = rospy.Subscriber("/tb" + str(robot_id) + "/scan", LaserScan, self.scan_callback)

		self.southnorth = [0.6, -0.25]
		self.eastwest = [0.25, 0.6]
		self.northsouth = [-0.6, 0.25]
		self.westeast = [-0.25, -0.6]

		(trans, rot) = self.get_robot_position()
		self.init_x, self.init_y = trans[0], trans[1]
		if (self.init_y <= 0 and self.init_y >= -0.5):
			self.intersection_x = self.southnorth[0]
			self.intersection_y = self.southnorth[1]
			self.INCOMING_DIRECTION = "North"
		elif (self.init_x <= 0.5 and self.init_x >= 0):
			self.intersection_x = self.eastwest[0]
			self.intersection_y = self.eastwest[1]
			self.INCOMING_DIRECTION = "West"
		elif (self.init_y <= 0.5 and self.init_y >= 0):
			self.intersection_x = self.northsouth[0]
			self.intersection_y = self.northsouth[1]
			self.INCOMING_DIRECTION = "South"
		else:
			self.intersection_x = self.westeast[0]
			self.intersection_y = self.westeast[1]
			self.INCOMING_DIRECTION = "East"

	def listen_callback(self, msg):
		safe_roads = msg.data.split(" ")
		print(safe_roads)
		self.SAFE_SOUTH_NORTH = True if safe_roads[0] == "1" else False
		self.SAFE_NORTH_SOUTH = True if safe_roads[1] == "1" else False
		self.SAFE_EAST_WEST = True if safe_roads[2] == "1" else False
		self.SAFE_WEST_EAST = True if safe_roads[3] == "1" else False
		# TODO Implement right turn policy, this is only to test if right turn is possible
		""" Delete this asap """
		# if self.INCOMING_DIRECTION == "North" or self.INCOMING_DIRECTION == "South":
		# 	if self.TURNING_DIRECTION == "Right":
		# 		if msg.data == "1 0":
		# 			self.SAFE_NORTH_SOUTH = True
		# 			self.SAFE_EAST_WEST = False
		# elif self.INCOMING_DIRECTION == "East" or self.INCOMING_DIRECTION == "West":
  #   			if self.TURNING_DIRECTION == "Right":
		# 		if msg.data == "0 1":
		# 			self.SAFE_NORTH_SOUTH = False
		# 			self.SAFE_EAST_WEST = True
		# else:
		# 	#TODO Think about edge cases?
		# 	pass

		# if self.INCOMING_DIRECTION == "North" or self.INCOMING_DIRECTION == "South":
		# 	if self.TURNING_DIRECTION == "Straight" or self.TURNING_DIRECTION == "Left":
		# 		if msg.data == "1 0":
		# 			self.SAFE_NORTH_SOUTH = True
		# 			self.SAFE_EAST_WEST = False
		# elif self.INCOMING_DIRECTION == "East" or self.INCOMING_DIRECTION == "West":
  #   			if self.TURNING_DIRECTION == "Straight" or self.TURNING_DIRECTION == "Left":
		# 		if msg.data == "0 1":
		# 			self.SAFE_NORTH_SOUTH = False
		# 			self.SAFE_EAST_WEST = True
		# else:
		# 	#TODO Think about edge cases?
		# 	pass
	
	def scan_callback(self, msg):
		minIndex = int(ceil((self.MIN_SCAN_ANGLE - msg.angle_min) / msg.angle_increment))
		maxIndex = int(floor((self.MAX_SCAN_ANGLE - msg.angle_min) / msg.angle_increment))
		for i in range(minIndex, maxIndex+1, 1):
			if msg.ranges[i] <= self.SAFETY_DISTANCE and not self.inside_central_intersection_area:
				self.FORWARD_SPEED = 0		
				# print("Stopping, object in front of me is %f away" % msg.ranges[i])
				return
		self.FORWARD_SPEED = 0.5

	def sendIntersectionArrivalTime(self):
    		time = str(rospy.get_time())
		id_and_time = self.robot_id + " " + time + " " + self.INCOMING_DIRECTION + " " + self.TURNING_DIRECTION + " " + self.RELATIVE_POSITION
		self.robot_relative_position.publish(id_and_time)

	def get_robot_position(self):
		while True:
			try:
				(trans, rot) = self.tf_listener.lookupTransform('/map', self.base_footprint, rospy.Time(0))
				(roll, pitch, yaw) = euler_from_quaternion(rot)
				# Converts all yaw to positive values in range 0-2pi
				yaw = fmod((2 * pi) + yaw, 2 * pi)
				# TODO: Robots moving in the north to south direction may have yaw that 
				# switches between 2pi and 0 instaneously. Force values between 340-360 to be 0 degrees
				# Can this be fixed by constantly calculating two values and working out which requires least
				# amount of change. E.g goal is 0 degrees but because we are slightly to the right of the goal
				# goal will be 6.1.  
				"""
				if yaw > self.correctionAngle and yaw <= 2 * pi:
					yaw = 0
				"""
				return (trans, yaw)
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

	# The faster the forward speed, the higher the turn_bias must be set.
	# Vice versa, if it is slowing (e.g isTurning) then the turn_bias can be reduced
	# At very low speeds, a high turn_bias can be seen as the robot pivoting on the spot
	# NB turn_bias can be thought of as a percentage, if it is set as 2. Then there is 50% 
	# chance of turning, if 3 66% chance of turning, if 4 75% chance of turning etc. 
	# This is because it is using modulo calculation, as such every second tick of the server
	# the robot is turning. If you a lower percentage is desired, please enter a negative number.
	# -2 will give 50%, -3 33%, 4 25% etc etc. Have not tested for -1, 0, 1 but behaviour could be
	# worked out
	def move(self, goal_x, goal_y, turn_bias, isturning):
		distance = sys.maxint
		angle, adjangle, adjcounter = 0, 0, 0
		cmd_vel_msg = Twist()

		while distance > 0.3:
			
			# if self.inside_central_intersection_area:
			# 	self.sendIntersectionArrivalTime()
			adjcounter += 1
			(trans, yaw) = self.get_robot_position()
			pos_x, pos_y = trans[0], trans[1]
			distance = sqrt(pow(pos_y - goal_y, 2) + pow(pos_x - goal_x, 2))
			target_angle = atan2(goal_y - pos_y, goal_x - pos_x)
			# Forces all angles (radians) to be positive between 0 and 2pi
			target_angle = fmod((2 * pi) + target_angle, 2 * pi)

			if turn_bias > 0:
				if adjcounter % turn_bias == 0:
					adjangle = 0
					adjcounter = 0
				else:
					adjangle = self.manage_turning_angle(target_angle, yaw)
			else:
				if adjcounter % -turn_bias == 0:
					adjangle = self.manage_turning_angle(target_angle, yaw)
					adjcounter = 0
				else:
					adjangle = 0
					
			cmd_vel_msg.angular.z = adjangle	

			if isturning == False:
				cmd_vel_msg.linear.x = self.FORWARD_SPEED
			else:
				cmd_vel_msg.linear.x = self.FORWARD_SPEED_IN_INTERSECTION
			# print("Publishing linear %f and angular %f" % (cmd_vel_msg.linear.x, cmd_vel_msg.angular.z))
			self.cmd_vel.publish(cmd_vel_msg)
			self.r.sleep()

		cmd_vel_msg.linear.x = 0.0;
		cmd_vel_msg.angular.z = 0.0;
		self.cmd_vel.publish(cmd_vel_msg);
		print("======robot no. %s stopped=========" % self.robot_id)

	def manage_turning_angle(self, target_angle, yaw):
		# Target angle in critical section
		if target_angle <= self.correctionAngle or target_angle >= (2*pi) - self.correctionAngle:
			# Yaw in critical section
			if abs(0 - yaw) <= abs((2 * pi) - yaw):
				adjangle = min(-2 * yaw, -self.MIN_ADJ_ANGLE)
				# adjangle = -2 * target_angle
			# Yaw not in critical section
			else:
				adjangle = max(2 * ((2 * pi) - yaw), self.MIN_ADJ_ANGLE)
		# Target angle not in critical section
		else:
			angle1 = 2 * (target_angle - yaw)
			angle2 = 2 * (target_angle - (2*pi - yaw))
			# Yaw in critical section
			if yaw >= (2*pi) - self.correctionAngle or yaw <= self.correctionAngle:
				if abs(angle1) < abs(angle2):
					adjangle = angle1
				else:
					adjangle = angle2
			# Yaw not in critical section
			else:
				adjangle = 2 * (target_angle - yaw)
		return adjangle


	def goToIntersection(self):
		self.move(self.intersection_x, self.intersection_y, 5, False)
		
	def go_left(self):
		self.TURNING_DIRECTION = "Left"
		self.RELATIVE_POSITION = "Begin_Intersection"
		self.sendIntersectionArrivalTime()

		self.wait()

		self.inside_central_intersection_area = True
		self.RELATIVE_POSITION = "In_Intersection"
		self.intersectionLeftTurn()
		self.RELATIVE_POSITION = "End_Intersection"
		self.sendIntersectionArrivalTime()
		self.inside_central_intersection_area = False

		if abs(self.init_x) > abs(self.init_y):
			if self.init_x < 0:
				self.move(-self.init_y, self.TOTALDISTANCETRAVELLED + self.init_x, 4, False)
			else:
				self.move(-self.init_y, -self.TOTALDISTANCETRAVELLED + self.init_x, 4, False)
		else:
			if self.init_y < 0:
				self.move(-(self.TOTALDISTANCETRAVELLED + self.init_y), self.init_x, 4, False)
			else:
				self.move(self.TOTALDISTANCETRAVELLED - self.init_y, self.init_x, 4, False)


	def intersectionLeftTurn(self):
		sign_x = 1 if self.intersection_x >= 0 else - 1
		sign_y = 1 if self.intersection_y >= 0 else - 1
		if abs(self.intersection_x) > abs(self.intersection_y):
			self.move(sign_x * abs(self.intersection_y), sign_y * 2 * abs(self.intersection_x), -2, True)
		else:
			self.move(sign_x * 2 * abs(self.intersection_y), sign_y * abs(self.intersection_x), -2, True)

	#TODO check order of end_intersection and message sernding
	def go_right(self):
		self.TURNING_DIRECTION = "Right"
		self.RELATIVE_POSITION = "Begin_Intersection"
		self.sendIntersectionArrivalTime()

		self.wait()

		self.inside_central_intersection_area = True
		self.RELATIVE_POSITION = "In_Intersection"
		self.move_up()
		self.intersectionRightTurn()
		self.RELATIVE_POSITION = "End_Intersection"
		self.sendIntersectionArrivalTime() #IS this correct?
		self.inside_central_intersection_area = False

		if (abs(self.init_x) > abs(self.init_y)):
			if self.init_x < 0:
				self.move(self.init_y, -(self.TOTALDISTANCETRAVELLED + self.init_x), 4, False)
			else:
				self.move(self.init_y, self.TOTALDISTANCETRAVELLED - self.init_x, 4, False)	
		else:
			if self.init_y < 0:
				self.move(self.TOTALDISTANCETRAVELLED + self.init_y, -self.init_x, 4, False)
			else:
				self.move(-self.TOTALDISTANCETRAVELLED + self.init_y, -self.init_x, 4, False)


	def move_up(self):
		(trans, rot) = self.get_robot_position()
		temp_x, temp_y = trans[0], trans[1]
		#South -> North
		if (temp_y <= 0 and temp_y >= -0.5):
			self.move(0, temp_y, 4, True)
		#East -> West
		elif (temp_x <= 0.5 and temp_x >= 0):
			self.move(temp_x, 0, 4, True)
		# North -> South
		elif (temp_y <= 0.5 and temp_y >= 0):
			self.move(0, temp_y, 4, True)
		# West -> East
		else:
			self.move(temp_x, 0, 4, True)

	def intersectionRightTurn(self):
		if abs(self.intersection_x) > abs(self.intersection_y):
			self.move(self.intersection_y, 2 * self.intersection_x, -3, True)        
		else:
			self.move(-2 * self.intersection_y, -self.intersection_x, -3, True)

	def go_straight(self):
		self.TURNING_DIRECTION = "Straight"
		self.RELATIVE_POSITION = "Begin_Intersection"
		self.sendIntersectionArrivalTime()

		self.wait()

		self.inside_central_intersection_area = True
		self.RELATIVE_POSITION = "In_Intersection"
		self.intersectionStraight()
		self.RELATIVE_POSITION = "End_Intersection"
		self.sendIntersectionArrivalTime()
		self.inside_central_intersection_area = False

		if abs(self.init_x) > abs(self.init_y):
			if self.init_x < 0:
				self.move(self.TOTALDISTANCETRAVELLED + self.init_x, self.init_y, 5, False)
			else:
				self.move(-self.TOTALDISTANCETRAVELLED + self.init_x, self.init_y, 5, False)
		else:
			if self.init_y < 0:
				self.move(self.init_x, self.TOTALDISTANCETRAVELLED + self.init_y, 5, False)
			else:
				self.move(self.init_x, -self.TOTALDISTANCETRAVELLED + self.init_y, 5, False)


	def intersectionStraight(self):
		if abs(self.intersection_x) < abs(self.intersection_y):
			self.move(self.intersection_x, -1.25 * self.intersection_y, 2, True)        
		else:
			self.move(-1.25 *self.intersection_x, self.intersection_y, 2, True)	# Make robot completely clear intersection before reporting it is good

	# TODO - when opposing road is active, so should the corresponding opposite road
	def wait(self):
		# if self.INCOMING_DIRECTION == "North":
		# 	while not self.SAFE_NORTH_SOUTH or not self.SAFE_SOUTH_NORTH:
		# 		self.r.sleep()
		# if self.INCOMING_DIRECTION == "South":
		# 	while not self.SAFE_SOUTH_NORTH or not not self.SAFE_NORTH_SOUTH:
		# 		self.r.sleep()
		# if self.INCOMING_DIRECTION == "East":
		# 	while not self.SAFE_EAST_WEST or not self.SAFE_WEST_EAST:
		# 		self.r.sleep()
		# if self.INCOMING_DIRECTION == "West":
		# 	while not self.SAFE_WEST_EAST or not self.SAFE_EAST_WEST:
		# 		self.r.sleep()
		if self.INCOMING_DIRECTION == "North":
			while not self.SAFE_SOUTH_NORTH:
				print("WAITING to got %s" % self.INCOMING_DIRECTION)
				self.r.sleep()
		if self.INCOMING_DIRECTION == "South":
			while not self.SAFE_NORTH_SOUTH:
				print("WAITING to got %s" % self.INCOMING_DIRECTION)
				self.r.sleep()
		if self.INCOMING_DIRECTION == "West":
			while not self.SAFE_EAST_WEST:
				print("WAITING to got %s" % self.INCOMING_DIRECTION)
				self.r.sleep()
		if self.INCOMING_DIRECTION == "East":
			while not self.SAFE_WEST_EAST:
				print("WAITING to got %s" % self.INCOMING_DIRECTION)
				self.r.sleep()

	

if __name__ == '__main__':
	try:
		tb_robot = Robot(sys.argv[1])
		tb_robot.goToIntersection()
		turn = randint(0, 1)
		# tb_robot.go_straight() if turn == 1 else tb_robot.go_left()
		# tb_robot.go_straight()
		# tb_robot.go_left()
		tb_robot.go_right()
	except rospy.ROSInterruptException:
		pass

	"""
	On first instance of successful delete_model, gzserver crashes making future attempts not work.
	Ideally, we can controll when robots spawn and despawns so we do not have to preload the models
	into the simulation hence reducing the CPU utilisation.

	Google:
	spawn_sdf_model crash after delete call
	gazebo crashes on model deletion

	# rospy.wait_for_service("gazebo/delete_model")
	# delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
	# delete_model("tb" + str(sys.argv[1])
	"""
