from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from custom_interfaces.srv import FindWall
import rclpy
from rclpy.node import Node
import numpy as np
import time

class WallFinderServer(Node):
    def __init__(self):
        super().__init__('wall_finder_server')
        
        self.group = ReentrantCallbackGroup()

        self.wall_finder_srv = self.create_service(
            FindWall, 
            "find_wall", 
            self.find_wall_callback, 
            callback_group=self.group)

        self.twist_publisher = self.create_publisher(
            Twist, 
            "cmd_vel",
            10,
            callback_group=self.group
        )
        
        self.sub_laser = self.create_subscription(
            LaserScan,
            "scan",
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self.group
        )
        self.initial_configuration = False
        self.turn_sign = +1
        self.laser_min_idx = 0
        self.laser_min = -1.0
        self.angle_to_rotate = 0.0
        self.laser_front = 0.0
        self.n_ranges = 0.0
        self.front_laser_idx = 0
        self.right_laser_idx = 0
        
        self.dist_tol = 0.03
        self.laser_idx_tol = 6
        
        self.vel_angular_z = 0.2
        self.vel_linear_x = 0.05

        self.first_turn_finished = False
        self.second_turn_finished = False

    def laser_callback(self, msg):
        
        if not self.initial_configuration:
            self.n_ranges = round((2 * np.pi / msg.angle_increment) + 1)
            self.front_laser_idx = int(self.n_ranges/2)
            self.right_laser_idx = int(self.n_ranges/4)
            self.initial_configuration = True
            self.get_logger().info('Front laser idx: "%s"' % str(self.front_laser_idx))
            self.get_logger().info('Right laser idx: "%s"' % str(self.right_laser_idx))
        
        self.laser_min_idx = np.argmin(msg.ranges[:])
        self.laser_min = msg.ranges[self.laser_min_idx]
        min_range_angle = (2 * np.pi * (self.laser_min_idx + 1)) / self.n_ranges
        if not self.first_turn_finished:
            self.angle_to_rotate = min_range_angle - np.pi
            if self.angle_to_rotate < 0:
                self.turn_sign = -1    
            self.get_logger().info('Angle to rotate: "%s"' % str(self.angle_to_rotate * 180 / np.pi))
        elif not self.second_turn_finished:
            self.angle_to_rotate = min_range_angle - np.pi/2
            self.get_logger().info('Angle to rotate 2: "%s"' % str(self.angle_to_rotate * 180 / np.pi))
        # self.get_logger().info('Angle to rotate: "%s"' % str(self.laser_min_idx ))
        self.laser_front = msg.ranges[self.front_laser_idx]
        
            
    def find_wall_callback(self, request, response):
        msg = Twist()
        
        while (self.laser_min < 0):
            self.get_logger().info('WAITING FOR IDENTIFYING THE NEAREST WALL')
            time.sleep(0.5)

        while (abs(self.laser_min_idx - self.front_laser_idx) > self.laser_idx_tol):
            msg.angular.z = self.turn_sign * self.vel_angular_z
            self.twist_publisher.publish(msg)
            time.sleep(0.5)
            self.get_logger().info('FIRST TURN')
        
        self.first_turn_finished = True
        msg.angular.z = 0.0
        self.twist_publisher.publish(msg)
        self.get_logger().info('First turn finished, the robot is looking the nearest wall')

        linear_x_sign = 1
        if self.laser_front < 0.3:
            linear_x_sign = -1
        while abs(self.laser_front - 0.3) > self.dist_tol:
            msg.linear.x = linear_x_sign * self.vel_linear_x
            msg.angular.z = 0.0
            self.twist_publisher.publish(msg)
            time.sleep(0.5)
            self.get_logger().info('LINEAR MOTION')
            
        msg.linear.x = 0.0
        self.twist_publisher.publish(msg)
        self.get_logger().info('The robot 30 cm away from the wall')

        while (abs(self.laser_min_idx - self.right_laser_idx) > self.laser_idx_tol):
            msg.angular.z = self.vel_angular_z
            self.twist_publisher.publish(msg)
            time.sleep(0.5)
            self.get_logger().info('SECOND TURN')

        self.second_turn_finished = True
        msg.angular.z = 0.0
        self.twist_publisher.publish(msg)
        self.get_logger().info('The robot is aligned with the right wall')

        response.wallfound = True
        
        return response


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    wall_finder_node = WallFinderServer()
    # Create a MultiThreadedExecutor with 3 threads
    executor = MultiThreadedExecutor(num_threads=3)
    # Add the node to the executor
    executor.add_node(wall_finder_node)
    try:
        # Spin the executor
        executor.spin()
    finally:
        # Shutdown the executor
        executor.shutdown()
        wall_finder_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





