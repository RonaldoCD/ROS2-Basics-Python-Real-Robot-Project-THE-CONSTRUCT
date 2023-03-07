import rclpy


from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from custom_interfaces.action import OdomRecord

import time
import numpy as np
import copy

class OdomRecordServer(Node):

    def __init__(self):
        super().__init__('record_odom_server')
        
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        
        self._action_server = ActionServer(self, 
            OdomRecord, 
            'record_odom',
            self.action_callback,
            callback_group=self.group1)

        self.odom_subscriber_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE),
            callback_group=self.group2)
        
        self.last_point = Point32()
        self.last_odom = Point32()
        self.first_odom = Point32()
        self.first_odom_identified = False
        self.total_distance = 0.0
        self.odom_record = []
        self.complete_lap_timer = self.create_timer(0.1, self.check_complete_lap, callback_group=self.group2)
        self.lap_finished = False
        self.first_movement = False

    def check_complete_lap(self):
        if self.first_movement:
            self.distance_to_start = np.sqrt((self.first_odom.x - self.last_odom.x)**2 + (self.first_odom.y - self.last_odom.y)**2)
            self.get_logger().info('Distance to start: "%s"' % str(self.distance_to_start))
            if self.distance_to_start <= 0.05:
                self.lap_finished = True
                self.last_point = copy.deepcopy(self.last_odom)
                self.get_logger().info('LAP FINISHED IS SET TO TRUE')
                

    def action_callback(self, goal_handle):
        
        self.get_logger().info('Executing goal...')

        feedback_msg = OdomRecord.Feedback()
        last_odom = self.first_odom
        self.get_logger().info('Last odom: "%s"' % str(last_odom))
        self.odom_record.append(last_odom)
        while not self.lap_finished:
            time.sleep(1)
            current_odom = self.last_odom
            # self.get_logger().info('Last odom x: "%s"' % str(last_odom.x))
            # self.get_logger().info('Last odom y: "%s"' % str(last_odom.y))
            # self.get_logger().info('Current odom x: "%s"' % str(current_odom.x))
            # self.get_logger().info('Current odom y: "%s"' % str(current_odom.y))

            delta_x = np.power(current_odom.x - last_odom.x, 2)
            delta_y = np.power(current_odom.y - last_odom.y, 2)
            # self.get_logger().info('Delta_x: "%s"' % str(delta_x))
            # self.get_logger().info('Delta_y: "%s"' % str(delta_y))
            dist = np.sqrt(delta_x + delta_y)   
            # self.get_logger().info('Dist: "%s"' % str(dist))
            # self.get_logger().info('Current odom: "%s"' % str(current_odom))
            self.total_distance += dist         
            self.get_logger().info('Current distance feedback: "%s"' % str(self.total_distance))
            feedback_msg.current_total = self.total_distance
            goal_handle.publish_feedback(feedback_msg)
            self.odom_record.append(current_odom)

            last_odom = copy.deepcopy(current_odom)

            if not self.first_movement:
                self.first_movement = True
                self.get_logger().info('Action server has started to check distance to start')
            
        self.odom_record.append(self.last_point)
        goal_handle.succeed()

        result = OdomRecord.Result()
        result.list_of_odoms = self.odom_record
        self.get_logger().info('The Action Server has finished, it has recorded: "%s" points' % str(len(self.odom_record)))
        return result
    
    def odom_callback(self, msg):
        # print the log info in the terminal
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y    
        if not self.first_odom_identified:
            self.first_odom_identified = True
            self.first_odom.x = x
            self.first_odom.y = y
            self.get_logger().info('First odom')
        else:
            self.last_odom.x = x
            self.last_odom.y = y
            # self.get_logger().info('Last odom')

    def get_odom(self):
        return self.last_odom
            

def main(args=None):
    rclpy.init(args=args)
    odom_recorder_node = OdomRecordServer()
    # Create a MultiThreadedExecutor with 3 threads
    executor = MultiThreadedExecutor(num_threads=3)
    # Add the node to the executor
    executor.add_node(odom_recorder_node)
    try:
        # Spin the executor
        executor.spin()
    finally:
        # Shutdown the executor
        executor.shutdown()
        odom_recorder_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()