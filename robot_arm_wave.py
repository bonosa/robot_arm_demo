#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class RobotArmWaveDemo(Node):
    def __init__(self):
        super().__init__('robot_arm_wave_demo')
        
        # Create publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer to publish joint states at 30Hz
        self.timer = self.create_timer(1.0/30.0, self.publish_joint_states)
        
        # Initialize time counter for animation
        self.start_time = time.time()
        
        # Joint names for a simple 6-DOF arm
        self.joint_names = [
            'base_joint',
            'shoulder_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        self.get_logger().info('Robot Arm Wave Demo Started! Check RViz to see the arm waving.')
        
    def publish_joint_states(self):
        # Calculate time since start
        current_time = time.time() - self.start_time
        
        # Create joint state message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        
        # Create waving motion
        wave_freq = 1.5  # Wave frequency in Hz
        wave_amplitude = 0.8  # Wave amplitude in radians
        
        # Joint positions for waving motion
        joint_state.position = [
            0.0,  # base_joint - stationary
            math.pi/4,  # shoulder_joint - raised up
            -math.pi/3,  # elbow_joint - bent
            wave_amplitude * math.sin(2 * math.pi * wave_freq * current_time),  # wrist_1 - waving
            0.0,  # wrist_2 - stationary  
            0.0   # wrist_3 - stationary
        ]
        
        # Publish the joint states
        self.joint_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    
    # Create and run the node
    wave_demo = RobotArmWaveDemo()
    
    try:
        rclpy.spin(wave_demo)
    except KeyboardInterrupt:
        print("\nStopping robot arm wave demo...")
    finally:
        wave_demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()