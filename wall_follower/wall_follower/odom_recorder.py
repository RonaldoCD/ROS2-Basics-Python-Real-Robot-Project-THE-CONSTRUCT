import rclpy


from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from custom_interfaces.action import OdomRecord

import time
import numpy as np

class OdomRecordServer(Node):

    def __init__(self):
        super().__init__('record_odom_server')
        
        self.group1 = ReentrantCallbackGroup()
        self.group2 = ReentrantCallbackGroup()
        
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
        
        self.last_odom = Point()
        self.first_odom = Point()
        self.first_odom_identified = False
        self.total_distance = 0.0
        self.odom_record = []
    
    def action_callback(self, goal_handle):
        
        self.get_logger().info('Executing goal...')

        feedback_msg = OdomRecord.Feedback()
        lap_finished = False
        last_odom = self.get_odom()
        self.odom_record.append(last_odom)
        while not lap_finished:
            time.sleep(1)
            current_odom = self.get_odom()
            self.total_distance += np.sqrt((current_odom.x - last_odom.x)**2 + (current_odom.y - last_odom.y)**2)            
            self.get_logger().info('Current distance feedback: "%s"' % str(self.total_distance))
            feedback_msg.current_total = self.total_distance
            goal_handle.publish_feedback(feedback_msg)
            self.odom_record.append(current_odom)

            distance_to_start = np.sqrt((self.first_odom.x - current_odom.x)**2 + (self.first_odom.y - current_odom.y)**2)
            
            if distance_to_start < 0.05:
                lap_finished = True
            else:
                last_odom = current_odom
            
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
        else:
            self.last_odom.x = x
            self.last_odom.y = y
    
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