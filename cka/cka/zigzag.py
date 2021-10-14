import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile


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
		self.pub = self.create_publisher(Twist, 'cmd_vel', qos)

	def run(self) -> None:

		twist = Twist()

		while True:
			twist.linear.x = 0.11
			twist.linear.y = 0.0
			twist.linear.z = 0.0
			twist.angular.x = 0.0
			twist.angular.y = 0.0
			twist.angular.z = 0.0
			sleep(0.1)
			self.pub.publish(twist)


def main(args=None) -> None:
	rclpy.init(args=args)
	
	node = zigzag()

	
	rclpy.spin(node)
	
	node.destroy_node()
	
	rclpy.shutdown()

if __name__ == "__main__":
    main()
