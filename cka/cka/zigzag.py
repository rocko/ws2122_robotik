import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
#from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import Spawn
#from turtlesim.srv import SetPen

import random
import math
from time import sleep

class zigzag(Node):
	def __init__(self) -> None:
		super().__init__("zigzag node")
		self.get_logger().info("ZZzzzzzzzzzzzigzag")

def main(args=None) -> None:
	rclpy.init(args=args)
	
	node = zigzag()
	node.setup()
	node.move_controller()
	
	rclpy.spin(node)
	
	node.destroy_node()
	
	rclpy.shutdown()

if __name__ == "__main__":
    main()
