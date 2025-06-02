#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Create callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Subscribe to goal pose from RViz2
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Create action client for Nav2
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Navigation node initialized')
        
    def goal_callback(self, msg: PoseStamped):
        """Handle incoming goal poses from RViz2"""
        self.get_logger().info('Received new goal pose')
        
        # Wait for Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return
            
        # Create goal
        goal = NavigateToPose.Goal()
        goal.pose = msg
        
        # Send goal and get future
        self.get_logger().info('Sending goal to Nav2')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        """Handle the goal response from Nav2"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            return
            
        self.get_logger().info('Goal accepted by Nav2')
        
        # Get result future
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        """Handle the navigation result"""
        status = future.result().status
        if status == 4:  # Succeeded
            self.get_logger().info('Navigation successful!')
        else:
            self.get_logger().warning(f'Navigation failed with status: {status}')

def main():
    rclpy.init()
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
