import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np
import time

class WallFollower(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('wall_follower')
        # create the subscriber object
        # in this case, the subscriptor will be subscribed on /scan topic with a queue size of 10 messages.
        # use the LaserScan module for /scan topic
        # send the received info to the listener_callback method.
        
        self.cmd_vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_subscriber_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE))  # is the most used to read LaserScan data and some sensor data.
        
        self.odom_subscriber_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE))  # is the most used to read LaserScan data and some sensor data.
        
        self.timer_period = 0.5
        self.publisher_timer = self.create_timer(self.timer_period, self.motion)

        self.distance_to_wall = 0.0
        self.rotation_z = 0.0
        
        self.distance_to_front_wall = 0.0

        self.linear_vel = 0.1
        self.angular_vel = 0.1
        self.shift_angle = 0.0
        self.angle_tol = 0.05 * np.pi / 180

        self.print_timer_period = 2.5
        self.print_timer = self.create_timer(self.print_timer_period, self.print_function)
        self.dist_tol = 0.005
                    
    def print_function(self):
        self.get_logger().info('Distance to wall: "%s"' % str(self.distance_to_wall))
        self.get_logger().info('Distance to front wall: "%s"' % str(self.distance_to_front_wall))
        self.get_logger().info('Angle direccion: "%s"' % str(self.rotation_z * 180/ np.pi))
        self.get_logger().info('Shift angle: "%s"' % str(self.shift_angle * 180/ np.pi))

    def motion(self):
        # print the data
        msg = Twist()
        if self.distance_to_wall > 0.3:
            msg.linear.x = self.linear_vel
            msg.angular.z = - self.angular_vel
            msg_to_print = "The robot is approaching the wall"
        elif self.distance_to_wall < 0.2:
            msg.linear.x = self.linear_vel
            msg.angular.z = self.angular_vel
            msg_to_print = "The robot is moving away to the wall"
        else:
            delta_dist_to_wall = abs(self.distance_to_wall - 0.25)
            turn_sign = 1
            if self.shift_angle > 0:
                turn_sign = -1

            if delta_dist_to_wall >= self.dist_tol:
                dist_to_go = delta_dist_to_wall / np.sin(abs(self.shift_angle))
                if dist_to_go / (self.timer_period * self.linear_vel) >= 1:
                    msg.linear.x = self.linear_vel
                else:
                    msg.linear.x = dist_to_go / (self.timer_period * self.linear_vel)
                
                if abs(self.shift_angle) / (self.timer_period * self.angular_vel) >= 1:
                    msg.angular.z = turn_sign * self.linear_vel
                else:
                    msg.angular.z = turn_sign * (abs(self.shift_angle) / (self.timer_period * self.angular_vel))

            else:
                msg.linear.x = self.linear_vel
                msg.angular.z = 0
            
            msg_to_print = "The robot is advancing forward"

            # msg.linear.x = self.linear_vel
            # if abs(self.shift_angle) > self.angle_tol:
            #     if self.shift_angle > 0:
            #         msg.angular.z = - self.angular_vel
            #     else:
            #         msg.angular.z = self.angular_vel
            # else:
            #     msg.angular.z = 0.0
            # msg_to_print = "The robot is advancing forward"
            
        self.get_logger().info(msg_to_print)

        if self.distance_to_front_wall < 0.5:
            msg.linear.x = self.linear_vel
            msg.angular.z = 1.2
            self.get_logger().info("The robot is turning left to avoid front wall")

        self.cmd_vel_publisher_.publish(msg)
        

        
        
        # Logic of move
        
    def laser_callback(self, msg):
        # print the log info in the terminal
        shift_angle = self.rotation_z % (np.pi/2)
        # self.get_logger().info('Shift angle 1: "%s"' % str(shift_angle* 180/ np.pi))
        if shift_angle >= np.pi/4 and shift_angle < np.pi/2:
            shift_angle -= (np.pi/2)
        
        self.shift_angle = shift_angle
        
        # self.get_logger().info('Shift angle 2: "%s"' % str(shift_angle* 180/ np.pi))
        
        range_angle = (3 * np.pi / 2) - shift_angle - np.pi
        # self.get_logger().info('Range angle: "%s"' % str(range_angle* 180/ np.pi))
        ranges_length = round((2 * np.pi / msg.angle_increment) + 1)
        # self.get_logger().info('Ranges length: "%s"' % str(ranges_length))
        range_idx = int(range_angle * ranges_length / (2 * np.pi))
        # self.get_logger().info('Ranges index: "%s"' % str(range_idx))
        self.distance_to_wall = msg.ranges[range_idx]
        # self.get_logger().info('Distance to wall: "%s"' % str(self.distance_to_wall))

        self.distance_to_front_wall = msg.ranges[360]
        # distance_0 = msg.ranges[0]
        # distance_180 = msg.ranges[180]
        # distance_360 = msg.ranges[360]
        # distance_540 = msg.ranges[540]
        # self.get_logger().info('Distance to front wall: "%s"' % str(self.distance_to_front_wall))
        # self.get_logger().info('0° : "%s"' % str(distance_0))
        # self.get_logger().info('90° : "%s"' % str(distance_180))
        # self.get_logger().info('180° : "%s"' % str(distance_360))
        # self.get_logger().info('270° : "%s"' % str(distance_540))
        
        # time.sleep(5)

        
    def odom_callback(self, msg):
        # print the log info in the terminal
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.x
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        _, _, yaw = self.euler_from_quaternion([x, y, z, w])
        # self.get_logger().info('Yaw angle: "%s"' % str(yaw * 180/ np.pi))

        rotation_z = yaw % np.pi
        if yaw < 0:
            rotation_z += np.pi
        
        self.rotation_z = rotation_z
        # self.get_logger().info('Angle direccion: "%s"' % str(rotation_z * 180/ np.pi))
    
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
    # declare the node constructor
    wall_follower_node = WallFollower()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(wall_follower_node)
    # Explicity destroy the node
    wall_follower_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()