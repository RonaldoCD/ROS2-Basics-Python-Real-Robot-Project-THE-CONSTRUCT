# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the MyCustomServiceMessage module from custom_interfaces_service interface
from custom_interfaces.srv import FindWall
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile

import numpy as np
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import time

class WallFinder(Node):

    def __init__(self):
        # Here you have the class constructor
        # self.group1 = MutuallyExclusiveCallbackGroup()
        # self.group2 = MutuallyExclusiveCallbackGroup()
        # self.group3 = MutuallyExclusiveCallbackGroup()
        self.group1 = ReentrantCallbackGroup()
        
        # call the class constructor to initialize the node as service_stop
        super().__init__('wall_finder_server')

        self.odom_subscriber_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE),
            callback_group=self.group1)  

        self.laser_subscriber_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE),
            callback_group=self.group1)  

        self.wall_finder_srv = self.create_service(
            FindWall, 
            'find_wall', 
            self.wall_finder_srv_callback,
            callback_group=self.group1)

        self.cmd_vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.nearest_wall_identified = False
        self.angle_fisrt_turn = 0.0
        self.angular_z = 0.1
        self.linear_x = 0.08
        self.angle_to_rotate = 0.0
        self.robot_yaw = 0.0
        self.angle_tol = 5 * np.pi / 180
        self.dist_tol = 0.1

        self.print_timer_period = 3
        # self.print_timer = self.create_timer(self.print_timer_period, self.print_function)

        self.first_turn_finished = False
        self.dist_nearest_wall = 0.0
        self.dist_to_wall = 0.0

    def laser_callback(self, msg):
        # print the log info in the terminal
        # self.get_logger().info(str(self.nearest_wall_identified))
        if not self.nearest_wall_identified:
            ranges_length = round((2 * np.pi / msg.angle_increment) + 1)
            ranges = msg.ranges[:]
            # for i in range(ranges_length):
            #     ranges.append(msg.ranges[i])
            ranges = np.array(ranges)
            min_range_idx = np.argmin(ranges)
            self.get_logger().info('Min range idx: "%s"' % str(min_range_idx))

            min_range_angle = (2 * np.pi * (min_range_idx + 1)) / ranges_length
            angle_to_rotate = min_range_angle - np.pi
            self.angle_to_rotate = angle_to_rotate
            self.nearest_wall_identified = True
            self.dist_nearest_wall = msg.ranges[min_range_idx]
        
        self.dist_to_wall = msg.ranges[360]
    
    def print_function(self):
        self.get_logger().info('Robot yaw angle: "%s"' % str(self.robot_yaw * 180/ np.pi))
        self.get_logger().info('Dist to wall: "%s"' % str(self.dist_to_wall))

    def wall_finder_srv_callback(self, request, response):
        msg = Twist()
        robot_initial_yaw = self.robot_yaw

        rotation_z = (robot_initial_yaw + self.angle_to_rotate)%(2*np.pi)
        quadrant_idx = round(rotation_z / (np.pi/2))
        objective_angle = (quadrant_idx * np.pi / 2)%(2*np.pi)

        self.get_logger().info('Initial yaw angle: "%s"' % str(robot_initial_yaw * 180/ np.pi))
        self.get_logger().info('Objective angle: "%s"' % str(objective_angle * 180/ np.pi))
        
        while not self.nearest_wall_identified:
            self.get_logger().info('WAITING FOR IDENTIFYING THE NEAREST WALL')
            time.sleep(0.5)
            
        if self.nearest_wall_identified is True:
            while abs(self.robot_yaw - objective_angle)%(2*np.pi) > self.angle_tol:
                if self.angle_to_rotate > 0:
                    msg.angular.z = self.angular_z
                elif self.angle_to_rotate < 0:
                    msg.angular.z = -self.angular_z
                self.cmd_vel_publisher_.publish(msg)
                # self.get_logger().info('Turning to closest wall')
            msg.angular.z = 0.0
            self.cmd_vel_publisher_.publish(msg)
            self.first_turn_finished = True
        
        # self.get_logger().info('The robot is looking to the closest wall')
        linear_x_sign = 1
        if self.dist_to_wall < 0.3:
            linear_x_sign = -1
        while abs(self.dist_to_wall - 0.3) > self.dist_tol:
            msg.linear.x = linear_x_sign * self.linear_x
            msg.angular.z = 0.0
            self.cmd_vel_publisher_.publish(msg)
            # self.get_logger().info('The robot is moving to be 30 cm away from the wall')
        msg.linear.x = 0.0
        self.cmd_vel_publisher_.publish(msg)

        final_objective_angle = (objective_angle + (np.pi / 2.0)) % (2*np.pi)
        self.get_logger().info('Last objective angle: "%s"' % str(final_objective_angle * 180/ np.pi))
        while abs(self.robot_yaw - final_objective_angle) % (2 * np.pi) > self.angle_tol:
            msg.linear.x = 0.0
            msg.angular.z = self.angular_z
            self.cmd_vel_publisher_.publish(msg)
            # self.get_logger().info('The robot is doing the final rotation')

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_publisher_.publish(msg)
        
        response.wallfound = True
        
        return response
  
    def odom_callback(self, msg):

        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        # self.get_logger().info("(" + str(x) + ", " + str(y) + ", " + str(z) + ", " + str(w) + ")")

        _, _, yaw = self.euler_from_quaternion([x, y, z, w])
        # self.get_logger().info('Yaw angle: "%s"' % str(yaw * 180/ np.pi))

        rotation_z = yaw % np.pi
        if yaw < 0:
            rotation_z += np.pi
        
        self.robot_yaw = rotation_z
        self.get_logger().info('Angle direccion: "%s"' % str(rotation_z * 180/ np.pi))
    
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    wall_finder_node = WallFinder()
    # Create a MultiThreadedExecutor with 3 threads
    executor = MultiThreadedExecutor(num_threads=4)
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