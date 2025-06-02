#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Create action client for NavigateToPose
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribe to goal pose topic (this will be published from rviz2 on another machine)
        self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10)
            
        self.get_logger().info('Navigation node initialized')

    def goal_callback(self, pose_msg: PoseStamped):
        """Handle incoming goal poses"""
        self.get_logger().info('Received new goal pose')
        self._send_goal(pose_msg)

    async def _send_goal(self, pose_msg: PoseStamped):
        # Wait for action server
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for navigate_to_pose action server...')

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg

        self.get_logger().info('Sending goal...')

        # Send goal and get future for result
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback)

        try:
            # Wait for goal response
            goal_handle = await send_goal_future
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                return

            self.get_logger().info('Goal accepted')
            
            # Get result future
            result_future = await goal_handle.get_result_async()
            status = result_future.result().status
            if status != 4:  # 4 = succeeded
                self.get_logger().error(f'Goal failed with status: {status}')
            else:
                self.get_logger().info('Goal succeeded!')
                
        except Exception as e:
            self.get_logger().error(f'Exception while processing goal: {str(e)}')

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Distance remaining: {feedback.distance_remaining:.2f}m')

def main(args=None):
    rclpy.init(args=args)
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
