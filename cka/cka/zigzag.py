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

class SCAN_DIRECTION(Enum):
	FRONT = 0
	LEFT = 1
	RIGHT = 2

class STATE(Enum):
	SCAN = 0
	FORWARD = 1
	TURN_LEFT = 2
	TURN_RIGHT = 2

class VELOCITY(Enum):
	LINEAR = 0.3
	ANGULAR = 0.7
	STOP = 0.0

class zigzag(Node):
	def __init__(self) -> None:
		super().__init__("zigzag")
		
		qos = QoSProfile(depth=10)

		# Initialise variables
		self.current_pose = [.0, .0, .0]
		self.previous_pose = [.0, .0, .0]
	
		self.current_velocity = [.0, .0]
		
		self.state = 0
		# 0 - Get new direction
		# 1 - Forward
		# 2 - Turning left
		# 3 - Turning right

		self.scan_resolution = 45
		# Scanning direction CCW
		self.scan_angles = [0, 315, 270, 235, 180, 135, 45]
		#self.scan_angles = [0, 45, 90, 135, 0, 235, 270, 315]  # dont measure behind bot # [0, 45, 90, 135, 180, 235, 270, 315]
		self.scan_ranges = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.evasion_angle = 0.0  # in rad
		self.safety_distance = 0.7
		self.skip = False

		self.laser_scan = {}


		self.init_scan_state = False
		self.init_odom_state = False

		# Initialise publishers
		self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)
		
		# Initialise subscribers
		self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_profile_sensor_data)
		self.cmd_vel_raw_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, qos)
		self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, qos)       
		# Initialise timers
		self.update_timer = self.create_timer(0.01, self.update_callback)  

		self.get_logger().info("ZZzzzzzzzzzzzigzag initialized.")

	# CALLBACK FUNCTIONS
	def scan_callback(self, msg) -> None:
		# Fires up getting sensor data (continuisly)
		# Sensor data is contained in "msg"
		# self.scan_ranges = msg.ranges  # Update sensor data
		#self.scan_ranges = msg.ranges[::self.scan_resolution]  # Update range measurements
		#self.is_obstructed = min(self.scan_ranges) < self.safety_distance  # Detect if obstructed or not
		
		#self.evasion_angle = 0.0
		#if self.is_obstructed:
		#	obstruction_index = self.scan_ranges.index(min(self.scan_ranges))
		#	evasion_index = (obstruction_index + (int) (len(self.scan_ranges) / 2)) % len(self.scan_ranges)
		#	evasion_angle = self.scan_angles[evasion_index]
		#	self.get_logger().info("obstruction_index %s" % obstruction_index)
		#	self.get_logger().info("evasion_index %s" % evasion_index)
		#	self.get_logger().info("evasion_angle %s %s" % (evasion_angle, (evasion_angle * (math.pi / 180.0))))
		#	self.evasion_angle = evasion_angle * (math.pi / 180.0)

		self.laser_scan = {
			# Wrong indexes. This is lidar turns CW direction with front at 0 deg
			#"front": min(min(msg.ranges[:44:1] + msg.ranges[315::1]), 1),
			#"left": min(min(msg.ranges[45:134:1]), 1),
			#"back": min(min(msg.ranges[135:234:1]), 1),
			#"right": min(min(msg.ranges[235:314:1]), 1),
			"front": min(min(msg.ranges[0:1]), 1), #+ msg.ranges[315::1]), 1),
			"right": min(min(msg.ranges[313:314]), 1),
			#"back": min(min(msg.ranges[135:234:1]), 1),
			"left": min(min(msg.ranges[43:44]), 1),			
		}

		self.init_scan_state = True

	def cmd_vel_callback(self, msg) -> None:
		# Fires upon a change on linear or angular velocity induced by Publisher "cmd_vel_pub"
		# Change is contained in "msg"
		self.current_velocity[0] = msg.linear.x  # Update linear velocity
		self.current_velocity[1] = msg.angular.z  # Update angular velocity
		#self.get_logger().info("cmd_vel_raw_callback %s" % msg)

	def odom_callback(self, msg):
		self.current_pose[0] = msg.pose.pose.position.x
		self.current_pose[1] = msg.pose.pose.position.y
		# quaternion bullshit convert
		x = msg.pose.pose.orientation.x
		y = msg.pose.pose.orientation.y
		z = msg.pose.pose.orientation.z
		w = msg.pose.pose.orientation.w
		self.current_pose[2] = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
		self.init_odom_state = True
		#self.get_logger().info("odom_callback %s" % msg)

	def update_callback(self) -> None:
		if self.init_scan_state:  # and self.init_odom_state:
			#if self.is_obstructed:
			#	self.get_logger().info("obstructed")
			#	self.update_cmd_vel(VELOCITY.STOP.value, self.turn())
			#	# Do not update evasion angle
			#	#self.skip = True
			#else:
			#	self.get_logger().info("not obstructed")
			#	self.update_cmd_vel(VELOCITY.LINEAR.value, VELOCITY.STOP.value)
			#	#self.skip = False
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

	def turn(self) -> float:
		#angle = self.current_pose[2] + self.evasion_angle
		angle = self.evasion_angle - self.current_pose[2]
		self.get_logger().info("new angle %s" % angle)

		ang_velocity = 0.0  # VELOCITY.ANGULAR.value
		if math.fabs(angle) > 0.01:
			if angle >= math.pi:
				ang_velocity = -VELOCITY.ANGULAR.value
			elif math.pi > angle and angle >= 0:
				ang_velocity = VELOCITY.ANGULAR.value
			elif 0 > angle and angle >= -math.pi:
				ang_velocity = -VELOCITY.ANGULAR.value
			elif angle > -math.pi:
				ang_velocity = VELOCITY.ANGULAR.value

		return ang_velocity

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
