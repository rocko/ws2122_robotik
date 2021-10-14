import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

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
		self.linear_velocity = 0.0
		self.angular_velocity = 0.0
		self.scan_ranges = []
		self.init_scan_state = False

		# Initialise publishers
		self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
		
		# Initialise subscribers
		self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=qos_profile_sensor_data)
        
		# Initialise timers
		self.update_timer = self.create_timer(0.010, self.update_callback)



		# Run!
		self.run()


	# CALLBACK FUNCTIONS
	def scan_callback(self, msg):
		self.scan_ranges = msg.ranges
		self.init_scan_state = True

	def update_callback(self):
		if self.init_scan_state is True:
			self.detect_obstacle()

	def detect_obstacle(self):
		twist = Twist()
		obstacle_distance = min(self.scan_ranges)
		safety_distance = 0.3  # unit: m

		if obstacle_distance > safety_distance:
			twist.linear.x = self.linear_velocity
			twist.angular.z = self.angular_velocity
		else:
			twist.linear.x = 0.0
			twist.angular.z = 0.0
			self.get_logger().info("Obstacles are detected nearby. Robot stopped.")

		self.cmd_vel_pub.publish(twist)

	def run(self) -> None:
		twist = Twist()

		#while True:
		twist.linear.x = 0.11
		twist.linear.y = 0.0
		twist.linear.z = 0.0
		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = 0.0
			#sleep(0.1)
		self.get_logger().info("publishing")
		self.cmd_vel_pub.publish(twist)
			#rclpy.spin_once(self)


def main(args=None) -> None:
	rclpy.init(args=args)
	
	node = zigzag()

	
	rclpy.spin(node)
	
	node.destroy_node()
	
	rclpy.shutdown()

if __name__ == "__main__":
    main()
