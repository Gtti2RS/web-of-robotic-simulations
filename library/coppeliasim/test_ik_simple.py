#!/usr/bin/env python3
"""
Simple IK test that only requires target pose
Uses current robot state from MoveIt's planning scene monitor
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
import math

class SimpleIKClient(Node):
    def __init__(self):
        super().__init__('simple_ik_client')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def compute_ik(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0, frame='base_link'):
        """
        Compute IK using current robot state from planning scene
        Only need to provide target pose!
        """
        req = GetPositionIK.Request()
        
        # Set group name
        req.ik_request.group_name = "ur_manipulator"
        
        # Leave robot_state empty - MoveIt will use current state from planning scene
        # Or set is_diff=True to indicate we want to use current state
        req.ik_request.robot_state.is_diff = False
        
        # Set target pose
        req.ik_request.pose_stamped.header.frame_id = frame
        req.ik_request.pose_stamped.pose.position.x = x
        req.ik_request.pose_stamped.pose.position.y = y
        req.ik_request.pose_stamped.pose.position.z = z
        req.ik_request.pose_stamped.pose.orientation.x = qx
        req.ik_request.pose_stamped.pose.orientation.y = qy
        req.ik_request.pose_stamped.pose.orientation.z = qz
        req.ik_request.pose_stamped.pose.orientation.w = qw
        
        # Set IK link
        req.ik_request.ik_link_name = "tool0"
        
        # Options
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 5
        
        self.get_logger().info(f'Computing IK for pose: ({x}, {y}, {z}) in frame {frame}')
        
        future = self.cli.call_async(req)
        return future

def main():
    print("="*60)
    print("  Simple IK Test - Only Target Pose Required")
    print("  (Uses current robot state from planning scene)")
    print("  Frame: base_link (absolute workspace coordinates)")
    print("="*60)
    
    # Always use base_link frame
    frame = "base_link"
    
    # Defaults for gripper pointing down
    default_x, default_y, default_z = 0.4, 0.2, 0.5
    default_qx, default_qy, default_qz, default_qw = 0.0, 0.707, 0.0, 0.707
    
    # Get target pose from user
    print(f"\nTarget Position:")
    try:
        x = float(input(f"  x (default {default_x}): ").strip() or default_x)
        y = float(input(f"  y (default {default_y}): ").strip() or default_y)
        z = float(input(f"  z (default {default_z}): ").strip() or default_z)
    except ValueError:
        print("Invalid input, using defaults")
        x, y, z = default_x, default_y, default_z
    
    print(f"\nTarget Orientation (quaternion):")
    print(f"  Defaults: [{default_qx}, {default_qy}, {default_qz}, {default_qw}] = gripper DOWN")
    try:
        qx = float(input(f"  x (default {default_qx}): ").strip() or default_qx)
        qy = float(input(f"  y (default {default_qy}): ").strip() or default_qy)
        qz = float(input(f"  z (default {default_qz}): ").strip() or default_qz)
        qw = float(input(f"  w (default {default_qw}): ").strip() or default_qw)
    except ValueError:
        print("Invalid input, using defaults")
        qx, qy, qz, qw = default_qx, default_qy, default_qz, default_qw
    
    # Initialize and call
    rclpy.init()
    client = SimpleIKClient()
    future = client.compute_ik(x, y, z, qx, qy, qz, qw, frame=frame)
    rclpy.spin_until_future_complete(client, future)
    
    try:
        response = future.result()
        print("\n" + "="*60)
        
        if response.error_code.val == 1:
            print("✅ IK SOLUTION FOUND!")
            print("="*60)
            
            # Extract just the 6 arm joints
            joint_solution = list(response.solution.joint_state.position[:6])
            
            print("\nJoint Solution (radians):")
            print(joint_solution)
            
            print("\nJoint Solution (degrees):")
            print([round(math.degrees(j), 2) for j in joint_solution])
        else:
            print("❌ IK FAILED")
            print("="*60)
            errors = {1: 'SUCCESS', -15: 'INVALID_GROUP_NAME', -31: 'NO_IK_SOLUTION',
                     -17: 'INVALID_ROBOT_STATE', -18: 'INVALID_LINK_NAME', -21: 'FRAME_TRANSFORM_FAILURE'}
            error_name = errors.get(response.error_code.val, 'UNKNOWN_ERROR')
            print(f"Error: {error_name} (code: {response.error_code.val})")
            
    except Exception as e:
        print(f"❌ Service call failed: {e}")
    
    print("="*60 + "\n")
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

