import rclpy
import numpy as np
import time

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from custom_interfaces.srv import FindWall
from custom_interfaces.action import OdomRecord

class WallFollower(Node):

    def __init__(self):
       
        super().__init__('wall_follower')

        self.group1 = ReentrantCallbackGroup()
        
        # Wall finder client 
        self.wall_finder_client = self.create_client(FindWall, 
                                                    'find_wall', 
                                                    callback_group=self.group1)

        # checks once per second if a Service matching the type and name of the Client is available.
        while not self.wall_finder_client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')
        
        self.find_wall_req = FindWall.Request()
        self.find_wall_response = FindWall.Response()
        # self.request_sent = False
        self.find_wall_started = False
        self.find_wall_completed = False
        self.wall_follower_can_start = False

        self.odom_recorder_client = ActionClient(self, 
            OdomRecord, 
            'record_odom', 
            callback_group=self.group1)
        self.action_called = False

        # Wall follower functionality
        self.cmd_vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_subscriber_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self.group1)  # is the most used to read LaserScan data and some sensor data.
        
        self.odom_subscriber_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE),
            callback_group=self.group1)  # is the most used to read LaserScan data and some sensor data.
        
        self.timer_period = 0.5
        self.publisher_timer = self.create_timer(self.timer_period, self.motion, callback_group=self.group1)

        self.distance_to_wall = 0.0
        self.rotation_z = 0.0
        
        self.distance_to_front_wall = 0.0

        self.linear_vel = 0.1
        self.angular_vel = 0.1
        self.shift_angle = 0.0
        self.angle_tol = 0.1 * np.pi / 180

        self.print_timer_period = 2.0
        self.print_timer = self.create_timer(self.print_timer_period, self.print_function, callback_group=self.group1)
        self.dist_tol = 0.005

        self.action_finished = False
    
    def send_request(self):
        # send the request
        self.get_logger().info('Service has started')
        self.find_wall_response = self.wall_finder_client.call(self.find_wall_req)
        self.get_logger().info('Send_request has finished')
        # self.get_logger().info('SERVICE HAS FINISHED')
     
    def print_function(self):
        self.get_logger().info('Distance to wall: "%s"' % str(self.distance_to_wall))
        self.get_logger().info('Distance to front wall: "%s"' % str(self.distance_to_front_wall))
        self.get_logger().info('Angle direccion: "%s"' % str(self.rotation_z * 180/ np.pi))
        self.get_logger().info('Shift angle: "%s"' % str(self.shift_angle * 180/ np.pi))

    def motion_to_center(self):
        angular_z = 0.0
        delta_dist_to_wall = self.distance_to_wall - 0.25

        turn_sign = 1
        if delta_dist_to_wall > 0:
            turn_sign = -1
        
        if abs(delta_dist_to_wall) >= self.dist_tol:
            dist_to_go = abs(delta_dist_to_wall) / np.sin(abs(self.shift_angle))
            # self.get_logger().info("outside tolerance")
            if dist_to_go / (self.timer_period * self.linear_vel) >= 1:
                linear_x = self.linear_vel
                angular_z = turn_sign * self.angular_vel
            else:
                linear_x = (dist_to_go / (self.timer_period * self.linear_vel))* self.linear_vel
                angular_z = (-1) * (self.shift_angle/2) / self.timer_period
        else:
            # self.get_logger().info("inside tolerance")
            linear_x = self.linear_vel
            if self.shift_angle >= self.angle_tol:
                angular_z = (-1) * self.shift_angle / self.timer_period

        return (linear_x, angular_z)
            

    def motion(self):

        if not self.find_wall_started:
            self.find_wall_started = True
            self.get_logger().info('SERVICE HAS STARTED')
            self.send_request()
            
        elif not self.find_wall_completed:
            if not self.find_wall_response.wallfound:
                time.sleep(self.timer_period)
            else:
                self.get_logger().info('SERVICE HAS FINISHED')
                self.find_wall_completed = True
                self.wall_follower_can_start = True
        
        elif self.wall_follower_can_start:
            # self.get_logger().info('WALL FOLLOWER HAS STARTED')
            if not self.action_called:
                self.get_logger().info('ACTION HAS BEGAN')
                self.action_called = True
                self.action_send_goal()

            msg = Twist()
            if not self.action_finished:     
                if self.distance_to_front_wall < 0.5:
                    msg.linear.x = self.linear_vel
                    msg.angular.z = self.angular_vel * 5
                    # msg_to_print = "The robot is turning left to avoid front wall"
                else:
                    if self.distance_to_wall > 0.3:
                        msg.linear.x = self.linear_vel
                        msg.angular.z = - self.angular_vel
                        # msg_to_print = "The robot is approaching the wall"
                    elif self.distance_to_wall < 0.2:
                        msg.linear.x = self.linear_vel
                        msg.angular.z = self.angular_vel
                        # msg_to_print = "The robot is moving away to the wall"
                    else:
                        linear_x, angular_z = self.motion_to_center()
                        msg.linear.x = linear_x
                        msg.angular.z = angular_z 
                        # msg_to_print = "The robot is advancing forward"
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                    
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
    
    def action_send_goal(self):
        goal_msg = OdomRecord.Goal()

        self.odom_recorder_client.wait_for_server()
        self._send_goal_future = self.odom_recorder_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        list_of_odoms = result.list_of_odoms
        self.get_logger().info('The Action Server has finished, it has recorded: "%s" points' % str(len(list_of_odoms)))
        self.get_logger().info('Points: "%s"' % str(list_of_odoms))
        self.action_finished = True
        # rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Total distance feedback "%s"' % str(feedback.current_total))
    

def main(args=None):
    rclpy.init(args=args)
    wall_follower_node = WallFollower()
    executor = MultiThreadedExecutor(num_threads=6)
    # Add the node to the executor
    executor.add_node(wall_follower_node)
    try:
        # Spin the executor
        executor.spin()
    finally:
        # Shutdown the executor
        executor.shutdown()
        wall_follower_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()