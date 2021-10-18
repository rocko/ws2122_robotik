import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import random
import math
from time import sleep
import numpy as np
from enum import Enum

# max Angular and Linear velocities
BURGER_MAX_LIN_VEL = .22
BURGER_MAX_ANG_VEL = 2.84
LIN_VEL_STEP_SIZE = .01
ANG_VEL_STEP_SIZE = .1

class VELOCITY(Enum):
	LINEAR = BURGER_MAX_LIN_VEL
	ANGULAR = BURGER_MAX_ANG_VEL
	STOP = 0.0

class zigzag(Node):
	def __init__(self) -> None:
		super().__init__("zigzag")
		
		qos = QoSProfile(depth=10)

		# Initialise variables
		self.safety_distance = 0.7
		self.laser_scan = {}
		self.init_scan_state = False

		# Initialise publishers
		self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)
		
		# Initialise subscribers
		self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_profile_sensor_data)

		# Initialise timers
		self.update_timer = self.create_timer(0.01, self.update_callback)  

		self.get_logger().info("ZZzzzzzzzzzzzigzag initialized.")

	# CALLBACK FUNCTIONS
	def scan_callback(self, msg) -> None:
		# Fires up getting sensor data (continuisly)
		# Sensor data is contained in "msg"
		self.laser_scan = {
			"front": min(min(msg.ranges[0:1]), 1), #+ msg.ranges[315::1]), 1),
			"right": min(min(msg.ranges[313:314]), 1),
			"left": min(min(msg.ranges[43:44]), 1),			
		}
		self.init_scan_state = True

	def update_callback(self) -> None:
		if self.init_scan_state:  # and self.init_odom_state:
			if self.laser_scan["right"] > self.safety_distance and self.laser_scan["front"] < self.safety_distance and self.laser_scan["left"] < self.safety_distance:
				self.update_cmd_vel(VELOCITY.LINEAR.value, -VELOCITY.ANGULAR.value)
			elif self.laser_scan["right"] > self.safety_distance and self.laser_scan["front"] > self.safety_distance and self.laser_scan["left"] < self.safety_distance:
				self.update_cmd_vel(VELOCITY.LINEAR.value, -VELOCITY.ANGULAR.value)
			# Straight
			elif self.laser_scan["right"] > self.safety_distance and self.laser_scan["front"] > self.safety_distance and self.laser_scan["left"] > self.safety_distance:
				self.update_cmd_vel(VELOCITY.LINEAR.value, VELOCITY.STOP.value)
			elif self.laser_scan["right"] < self.safety_distance and self.laser_scan["front"] > self.safety_distance and self.laser_scan["left"] < self.safety_distance:
				self.update_cmd_vel(VELOCITY.LINEAR.value, VELOCITY.STOP.value)
			elif self.laser_scan["right"] < self.safety_distance and self.laser_scan["front"] > self.safety_distance and self.laser_scan["left"] > self.safety_distance:
				self.update_cmd_vel(VELOCITY.LINEAR.value, VELOCITY.ANGULAR.value)
			elif self.laser_scan["right"] > self.safety_distance and self.laser_scan["front"] < self.safety_distance and self.laser_scan["left"] > self.safety_distance:
				self.update_cmd_vel(VELOCITY.LINEAR.value, VELOCITY.STOP.value)
			elif self.laser_scan["right"] < self.safety_distance and self.laser_scan["front"] < self.safety_distance and self.laser_scan["left"] > self.safety_distance:
				self.update_cmd_vel(VELOCITY.LINEAR.value, VELOCITY.STOP.value)
			elif self.laser_scan["right"] < self.safety_distance and self.laser_scan["front"] < self.safety_distance and self.laser_scan["left"] < self.safety_distance:
				self.update_cmd_vel(VELOCITY.STOP.value, -VELOCITY.ANGULAR.value)

	def update_cmd_vel(self, lin_velocity:float, ang_velocity:float) -> None:
		twist = Twist()
		twist.linear.x = lin_velocity  # self.velocity[0]
		twist.angular.z = ang_velocity  #self.velocity[1]
		self.cmd_vel_pub.publish(twist)

def main(args=None):
	rclpy.init(args=args)
	node = zigzag()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
    main()
