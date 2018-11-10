#!/usr/bin/env python

import move_robot
import rospy
import sys
import os
import time
from collections import deque
from random import randint

class RandomController:

	def __init__(self, input):
		rospy.init_node("Random_controller", anonymous=True)
		self.sn_deque = deque()
		self.ew_deque = deque()
		self.ns_deque = deque()
		self.we_deque = deque()

		self.r = rospy.Rate(10)
		self.total_count = input

		for i in range(input):
			result = i % 4
			if result == 0:
				self.sn_deque.append(i)
			elif result == 1:
				self.ew_deque.append(i)
			elif result == 2:
				self.ns_deque.append(i)
			elif result == 3:
				self.we_deque.append(i)
			else:
				print("Error occurred, should not reach here")

	def num_robots_remain(self):
		return (len(self.sn_deque) + len(self.ew_deque) + len(self.ns_deque) + len(self.we_deque))

	def move_robot(self, input):
		str_arg = "python move_robot.py "
		if input == 0:
			try:
				str_arg += str(self.sn_deque.popleft())
				os.system(str_arg + " &")
			except Exception as e:
				# print e.message
				pass
		elif input == 1:
			try:
				str_arg += str(self.ew_deque.popleft())
				os.system(str_arg + " &")
			except Exception as e:
				# print e.message
				pass
		elif input == 2:
			try:
				str_arg += str(self.ns_deque.popleft())
				os.system(str_arg + " &")
			except Exception as e:
				# print e.message
				pass
		elif input == 3:
			try:
				str_arg += str(self.we_deque.popleft())
				os.system(str_arg + " &")
			except Exception as e:
				# print e.message
				pass
		else:
			print("Error occured, should not reach here")

if __name__ == "__main__":
	# Controller = RandomController(int(sys.argv[1]))
	# while Controller.num_robots_remain() > 0:
	# 	robot_signal, random_delay = randint(0, 3), randint(0, 2)
	# 	Controller.move_robot(robot_signal)
	# 	time.sleep(random_delay)
	Controller = RandomController(int(sys.argv[1]))
	Controller.move_robot(1)
	Controller.move_robot(3)