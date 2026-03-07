#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tier4_rtc_msgs.srv import CooperateCommands
from tier4_rtc_msgs.msg import CooperateCommand, Command, Module
from unique_identifier_msgs.msg import UUID

class RTCApprover(Node):
    def __init__(self):
        super().__init__('rtc_approver')
        
        # 创建服务客户端
        self.left_client = self.create_client(
            CooperateCommands,
            '/planning/cooperate_commands/static_obstacle_avoidance_left'
        )
        self.right_client = self.create_client(
            CooperateCommands,
            '/planning/cooperate_commands/static_obstacle_avoidance_right'
        )
        
        # 订阅状态
        self.create_subscription(
            CooperateStatusArray,
            '/planning/cooperate_status/static_obstacle_avoidance_left',
            self.left_status_callback,
            10
        )
        
    def left_status_callback(self, msg):
        # 当接收到需要批准的请求时，自动批准
        for status in msg.statuses:
            if status.requested:
                self.approve_command('left', status.uuid)
    
    def approve_command(self, direction, uuid):
        """发送批准命令"""
        client = self.left_client if direction == 'left' else self.right_client
        
        request = CooperateCommands.Request()
        request.stamp = self.get_clock().now().to_msg()
        
        cmd = CooperateCommand()
        cmd.module.type = Module.AVOIDANCE_LEFT if direction == 'left' else Module.AVOIDANCE_RIGHT
        cmd.command.type = Command.ACTIVATE
        cmd.uuid = uuid
        
        request.commands.append(cmd)
        
        if client.wait_for_service(timeout_sec=1.0):
            future = client.call_async(request)
            self.get_logger().info(f'Approved {direction} avoidance with UUID: {uuid}')
        else:
            self.get_logger().warn(f'{direction} service not available')

def main():
    rclpy.init()
    node = RTCApprover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
