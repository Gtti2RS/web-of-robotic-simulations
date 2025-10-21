#!/usr/bin/env python3
"""
Bridge node that converts CoppeliaSim joint data to sensor_msgs/JointState
CoppeliaSim publishes joint positions using std_msgs/String (JSON format)
This node converts it to proper JointState messages
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import json

class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')
        
        # Subscribe to CoppeliaSim joint data (as String/JSON)
        self.subscription = self.create_subscription(
            String,
            '/coppelia/ur10/joint_states_string',  # Namespaced CoppeliaSim joint states
            self.listener_callback,
            10)
        
        # Publish standard JointState to namespaced topic
        self.publisher = self.create_publisher(JointState, '/coppelia/ur10/joint_states', 10)
        
        self.get_logger().info('Joint State Bridge started')
        self.get_logger().info('Waiting for /coppelia/ur10/joint_states_string...')

    def listener_callback(self, msg):
        try:
            # Parse JSON from CoppeliaSim
            data = json.loads(msg.data)
            
            # Create JointState message
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = data.get('names', [])
            joint_state.position = data.get('positions', [])
            joint_state.velocity = data.get('velocities', [])
            joint_state.effort = data.get('efforts', [])
            
            # Publish
            self.publisher.publish(joint_state)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    bridge = JointStateBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

