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
	ANGULAR = 1.5
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

		self.scan_ranges = [.0, .0, .0]

		self.init_scan_state = False
		self.init_odom_state = False

		# Initialise publishers
		self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)
		
		# Initialise subscribers
		self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_profile_sensor_data)
		self.cmd_vel_raw_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, qos)
		self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, qos)       
		# Initialise timers
		self.update_timer = self.create_timer(0.010, self.update_callback)


		# Sleep 5 seconds until all the shit is loaded
		#sleep(5)  # seconds
		self.get_logger().info("ZZzzzzzzzzzzzigzag initialized.")

	# CALLBACK FUNCTIONS
	def scan_callback(self, msg) -> None:
		# Fires up getting sensor data (continuisly)
		# Sensor data is contained in "msg"
		# self.scan_ranges = msg.ranges  # Update sensor data
		scan_angles = [0, 30, 330]
		for i in range(len(scan_angles)):
			angle = scan_angles[i]  # 0, 30, 330 
			distance_at_angle = msg.ranges[angle]
			if distance_at_angle == float("inf"):
				self.scan_ranges[i] = 3.5
			else:
				self.scan_ranges[i] = distance_at_angle
			
			self.init_scan_state = True
		self.get_logger().info("scan_callback %s" % msg)
		self.get_logger().info("scan_callback %s" % self.scan_ranges)

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
		self.get_logger().info("odom_callback %s" % msg)

	def update_callback(self) -> None:
		if self.init_scan_state and self.init_odom_state:
			self.get_logger().info("STATE %s" % self.state)
			# Statemachine
			if self.state == STATE.SCAN.value:  # Is looking for new direction ?
				if self.scan_ranges[SCAN_DIRECTION.FRONT.value] > .7:  # Check if obstacle in front
					self.get_logger().info("Nothing in front")
					# Noting in front
					if self.scan_ranges[SCAN_DIRECTION.LEFT.value] < .6:  # Check if obstacle to the left
						self.get_logger().info("Obstacle to left")
						self.previous_pose = self.current_pose
						self.state = STATE.TURN_RIGHT.value  # Turn right
					elif self.scan_ranges[SCAN_DIRECTION.RIGHT.value] < .6:  # Check if obstacle to the right
						self.get_logger().info("Obstacle to right")
						self.previous_pose = self.current_pose
						self.state = STATE.TURN_LEFT.value  # Turn left
					else:
						# Drive forwards
						self.state = STATE.FORWARD.value

				if self.scan_ranges[SCAN_DIRECTION.FRONT.value] < .7: 	# Check if obstacle in front
					self.get_logger().info("Obstacle in front")
					self.previous_pose = self.current_pose					
					self.state = STATE.TURN_RIGHT.value  # Turn right
	
			elif self.state == STATE.FORWARD.value:  # Drive forwards
				self.update_cmd_vel(VELOCITY.LINEAR.value, VELOCITY.STOP.value)
				self.state = STATE.SCAN.value

			elif self.state == STATE.TURN_LEFT.value:  # Turn Left State
				self.get_logger().info("Prev Pose %s" % self.previous_pose)
				self.get_logger().info("Cur Pose %s" % self.current_pose)
				self.get_logger().info("Result %s" % math.fabs(self.previous_pose[2] - self.current_pose[2]))
				
				if math.fabs(self.previous_pose[2] - self.current_pose[2]) >= .5:  # Check if robot has turned away enough # 30 degrees rad
					self.state = STATE.SCAN.value
				else:
					self.update_cmd_vel(VELOCITY.STOP.value, VELOCITY.ANGULAR.value)  # Keep turning
			
			elif self.state == STATE.TURN_RIGHT.value:  # Turn Right State
				self.get_logger().info("Prev Pose %s" % self.previous_pose)
				self.get_logger().info("Cur Pose %s" % self.current_pose)
				self.get_logger().info("Result %s" % math.fabs(self.previous_pose[2] - self.current_pose[2]))
				if math.fabs(self.previous_pose[2] - self.current_pose[2]) >= .5:
					self.state = STATE.SCAN.value
				else:
					self.update_cmd_vel(VELOCITY.STOP.value, -VELOCITY.ANGULAR.value)
			else:
				self.state = STATE.SCAN.value

	def update_cmd_vel(self, lin_velocity, ang_velocity) -> None:
		twist = Twist()
		twist.linear.x = lin_velocity  # self.velocity[0]
		twist.angular.z = ang_velocity  #self.velocity[1]
		self.cmd_vel_pub.publish(twist)
			


	#def constrain(self, velocity:float, min:float, max:float) -> float:
	#	if velocity < min:
	#		velocity = min
	#	elif velocity > max:
	#		velocity = max
	#	
	#	return velocity

	#def speed_profile(self, cur_linear_velocity:float, new_linear_velocity:float, slope:float) -> float:
	#	if new_linear_velocity > cur_linear_velocity:
	#		cur_linear_velocity = min(new_linear_velocity, cur_linear_velocity + slope)
	#	elif new_linear_velocity < cur_linear_velocity:
	#		cur_linear_velocity = max(new_linear_velocity, cur_linear_velocity - slope)
	#	else:
	#		cur_linear_velocity = new_linear_velocity
	#
	#	return cur_linear_velocity

	#def get_obstruction(self, safety_distance:float=0.3, step:int=2) -> None:
		# front slice 315 - 45
		# right slice 45 - 135
		# back slice 135 - 225
		# left slice 225 - 315
		#self.obstruction[
		#	min(self.scan_ranges[:45:step]) < safety_distance or min(self.scan_ranges[315::step]), 
		#	min(self.scan_ranges[45:135:step]) < safety_distance, 
		#	min(self.scan_ranges[135:225:step]) < safety_distance, 
		#	min(self.scan_ranges[225:315:step]) < safety_distance
		#]
		#if min(self.scan_ranges[:45:step]) < safety_distance or min(self.scan_ranges[315::step]):
		#	self.obstruction[0] = True
		#if min(self.scan_ranges[45:135:step]) < safety_distance:
		#	self.obstruction[1] = True
		#if min(self.scan_ranges[135:225:step]) < safety_distance:
		#	self.obstruction[2] = True
		#if min(self.scan_ranges[225:315:step]) < safety_distance:
		#	self.obstruction[3] = True

	#	if self.scan_ranges[315] < safety_distance or self.scan_ranges[0] < safety_distance or self.scan_ranges[45] < safety_distance:
	#		self.obstruction[0] = True
	
	#def debug_obstruction(self):
	#
	#	self.get_logger().info("cmd_vel_raw_callback %s" % msg)

	#def detect_obstacle(self) -> None:
	#	twist = Twist()  # create Twist message
	#	obstacle_distance = self.scan_ranges[0]  # min(self.scan_ranges)  # detect the closest detected range to any object within 30 cm
	#	safety_distance = 0.3  # unit: m

	#	if obstacle_distance > safety_distance:
	#		twist.linear.x = self.velocity[0]
	#		twist.angular.z = self.velocity[1]
	#	else:
	#		twist.linear.x = 0.0
	#		twist.angular.z = self.velocity[1]			
		'''if not self.obstruction[0]:
			if self.velocity[0] < BURGER_MAX_LIN_VEL:
				twist.linear.x = self.speed_profile(self.velocity[0], self.constrain(self.velocity[0] + LIN_VEL_STEP_SIZE, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL), (LIN_VEL_STEP_SIZE / 2.0))  # Accelerate if needed
				twist.angular.z = self.velocity[1]  # Dont change angular velocity
			else:
				twist.linear.x = self.velocity[0]  # Dont change linear velocity
				twist.angular.z = 0.0  # self.velocity[1]  # Dont change angular velocity
		else:
			self.get_logger().info("Obstacles are detected nearby. Robot stopped.")
			twist.linear.x = 0.0
			# Obstructed in front?
			#if obstacle_distance < safety_distance:
			# Turn until obstruction in fron is no longer present
			##twist.angular.z = self.speed_profile(self.velocity[1], self.constrain(self.velocity[1] + ANG_VEL_STEP_SIZE, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL), (ANG_VEL_STEP_SIZE / 2.0))
		'''
			



	#	self.cmd_vel_pub.publish(twist)


def main(args=None):
	rclpy.init(args=args)
	
	node = zigzag()

	
	rclpy.spin(node)
	
	node.destroy_node()
	
	rclpy.shutdown()

if __name__ == "__main__":
    main()
