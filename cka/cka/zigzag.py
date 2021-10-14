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

# max Angular and Linear velocities
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

class zigzag(Node):
	def __init__(self) -> None:
		super().__init__("zigzag")
		self.get_logger().info("ZZzzzzzzzzzzzigzag")
		qos = QoSProfile(depth=10)

		# Initialise variables
		#self.odometry = Odometry()
		#self.last_pose = [0.0, 0.0, 0.0]
		#self.goal_pose = [0.0, 0.0, 0.0]
		self.state = 0
		self.velocity = [0.0, 0.0]
		self.scan_ranges = []
		self.init_scan_state = False
		#self.init_odom_state = False

		# Initialise publishers
		self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
		
		# Initialise subscribers
		self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=qos_profile_sensor_data)
		self.cmd_vel_raw_sub = self.create_subscription(Twist, 'cmd_vel_raw', self.cmd_vel_raw_callback, qos)
		#self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, qos)       
		# Initialise timers
		self.update_timer = self.create_timer(0.010, self.update_callback)


		# Sleep 5 seconds until all the shit is loaded
		sleep(5)  # seconds

	# CALLBACK FUNCTIONS
	def scan_callback(self, msg) -> None:
		# Fires up getting sensor data (continuisly)
		# Sensor data is contained in "msg"
		self.scan_ranges = msg.ranges  # Update sensor data

		self.init_scan_state = True

	def cmd_vel_raw_callback(self, msg) -> None:
		# Fires upon a change on linear or angular velocity induced by Publisher "cmd_vel_pub"
		# Change is contained in "msg"
		self.velocity[0] = msg.linear.x  # Update linear velocity
		self.velocity[1] = msg.angular.z  # Update angular velocity

	#def odom_callback(self, msg):
	#	self.last_pose[0] = msg.pose.pose.position.x
	#	self.last_pose[1] = msg.pose.pose.position.y
	#	_, _, self.last_pose[2] = self.euler_from_quaternion(msg.pose.pose.orientation)
	#	self.init_odom_state = True

	def update_callback(self) -> None:
		if self.init_scan_state is True:
			self.detect_obstacle()

	def detect_obstacle(self) -> None:
		twist = Twist()
		obstacle_distance = min(self.scan_ranges)
		safety_distance = 0.3  # unit: m

		# TODO: If turning skip detection part
		# if state == "turning":
		# else:
		#  # if no
		if obstacle_distance > safety_distance:
			if twist.linear.x < BURGER_MAX_LIN_VEL:
				self.get_logger().info("Robot accelerating.")
				twist.linear.x = self.velocity[0] + 0.01  # Accelerate if needed
			else:
				twist.linear.x = self.velocity[0]  # Dont change linear velocity
			
			twist.angular.z = self.velocity[1]  # Dont change angular velocity
			

		else:
			# Obstacle detected: Stop turtlebot
			# Try:
			# Rotate 45 degrees left
			# Rotate 90 degrees right
			# Rotate 135 degrees left
			# Rotate 180 degrees right
			twist.linear.x = 0.0
			twist.angular.z = 0.0
			self.get_logger().info("Obstacles are detected nearby. Robot stopped.")

			self.get_logger().info("Sleep 1 second")

			self.get_logger().info("Turn")

		self.cmd_vel_pub.publish(twist)

def main(args=None) -> None:
	rclpy.init(args=args)
	
	node = zigzag()

	
	rclpy.spin(node)
	
	node.destroy_node()
	
	rclpy.shutdown()

if __name__ == "__main__":
    main()
