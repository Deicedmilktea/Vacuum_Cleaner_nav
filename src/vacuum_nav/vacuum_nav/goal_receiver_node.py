#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException


class GoalReceiverNode(Node):
    """
    接收来自rviz2的目标点并发送给nav2进行导航
    """
    
    def __init__(self):
        super().__init__('goal_receiver_node')
        
        # 创建action client用于发送导航目标
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 订阅rviz2发布的目标点
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # rviz2默认发布目标点的topic
            self.goal_callback,
            10
        )
        
        # TF buffer和listener用于坐标变换
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Goal Receiver Node initialized')
        self.get_logger().info('Waiting for goal poses on /goal_pose topic...')
        
    def goal_callback(self, msg):
        """
        接收目标点回调函数
        """
        self.get_logger().info(f'Received goal pose: x={msg.pose.position.x:.2f}, '
                              f'y={msg.pose.position.y:.2f}, '
                              f'frame={msg.header.frame_id}')
        
        # 确保目标点在map坐标系下
        goal_pose = self.transform_pose_to_map(msg)
        
        if goal_pose is not None:
            self.send_goal(goal_pose)
        else:
            self.get_logger().error('Failed to transform goal pose to map frame')
    
    def transform_pose_to_map(self, pose_stamped):
        """
        将目标点变换到map坐标系
        """
        if pose_stamped.header.frame_id == 'map':
            return pose_stamped
        
        try:
            # 等待变换可用
            self.tf_buffer.can_transform('map', pose_stamped.header.frame_id, 
                                       pose_stamped.header.stamp, 
                                       timeout=rclpy.duration.Duration(seconds=1.0))
            
            # 执行坐标变换
            transformed_pose = self.tf_buffer.transform(pose_stamped, 'map')
            return transformed_pose
            
        except TransformException as ex:
            self.get_logger().error(f'Could not transform pose: {ex}')
            return None
    
    def send_goal(self, goal_pose):
        """
        发送导航目标给nav2
        """
        # 等待action server可用
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigate to pose action server not available!')
            return
        
        # 创建导航目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(f'Sending navigation goal to: '
                              f'x={goal_pose.pose.position.x:.2f}, '
                              f'y={goal_pose.pose.position.y:.2f}')
        
        # 发送目标
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """
        目标响应回调
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        
        # 获取结果
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """
        获取导航结果回调
        """
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
    
    def feedback_callback(self, feedback_msg):
        """
        导航反馈回调
        """
        feedback = feedback_msg.feedback
        # 可以在这里处理导航过程中的反馈信息
        # self.get_logger().info(f'Navigation feedback: {feedback}')


def main(args=None):
    rclpy.init(args=args)
    
    goal_receiver_node = GoalReceiverNode()
    
    try:
        rclpy.spin(goal_receiver_node)
    except KeyboardInterrupt:
        pass
    finally:
        goal_receiver_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
