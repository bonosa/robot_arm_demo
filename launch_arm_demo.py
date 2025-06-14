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
        
        # Animation state
        self.wave_style = 0  # Different wave patterns
        self.transition_time = 0
        
        self.get_logger().info('🤖 Robot Arm Wave Demo Started! Check RViz to see the arm waving.')
        self.get_logger().info('The robot will cycle through different wave styles!')
        
    def publish_joint_states(self):
        # Calculate time since start
        current_time = time.time() - self.start_time
        
        # Change wave style every 5 seconds
        if current_time - self.transition_time > 5.0:
            self.wave_style = (self.wave_style + 1) % 4
            self.transition_time = current_time
            styles = ['👋 Friendly Wave', '🎉 Excited Wave', '🌊 Smooth Wave', '✨ Royal Wave']
            self.get_logger().info(f'Switching to: {styles[self.wave_style]}')
        
        # Create joint state message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        
        # Different wave patterns based on style
        if self.wave_style == 0:  # Friendly wave
            wave_freq = 1.5
            wave_amplitude = 0.8
            base_rotation = 0.3 * math.sin(0.5 * current_time)
            shoulder_angle = math.pi/4 + 0.1 * math.sin(2 * current_time)
            elbow_angle = -math.pi/3
            wrist_wave = wave_amplitude * math.sin(2 * math.pi * wave_freq * current_time)
            wrist_2 = 0.0
            wrist_3 = 0.2 * math.sin(3 * current_time)
            
        elif self.wave_style == 1:  # Excited wave
            wave_freq = 3.0
            wave_amplitude = 1.0
            base_rotation = 0.5 * math.sin(current_time)
            shoulder_angle = math.pi/3 + 0.2 * math.sin(4 * current_time)
            elbow_angle = -math.pi/2.5 + 0.2 * math.sin(3 * current_time)
            wrist_wave = wave_amplitude * math.sin(2 * math.pi * wave_freq * current_time)
            wrist_2 = 0.3 * math.sin(5 * current_time)
            wrist_3 = 0.3 * math.sin(4 * current_time)
            
        elif self.wave_style == 2:  # Smooth wave
            wave_freq = 0.8
            wave_amplitude = 0.6
            base_rotation = 0.2 * math.sin(0.3 * current_time)
            shoulder_angle = math.pi/4 + 0.15 * math.sin(0.5 * current_time)
            elbow_angle = -math.pi/3 + 0.1 * math.sin(0.7 * current_time)
            wrist_wave = wave_amplitude * (math.sin(2 * math.pi * wave_freq * current_time) + 
                                          0.3 * math.sin(4 * math.pi * wave_freq * current_time))
            wrist_2 = 0.1 * math.sin(current_time)
            wrist_3 = 0.0
            
        else:  # Royal wave (style 3)
            wave_freq = 1.0
            t = current_time * wave_freq
            # Figure-8 pattern for the wrist
            base_rotation = 0.0
            shoulder_angle = math.pi/4
            elbow_angle = -math.pi/3
            wrist_wave = 0.5 * math.sin(2 * math.pi * t)
            wrist_2 = 0.3 * math.sin(4 * math.pi * t)
            wrist_3 = 0.4 * math.sin(2 * math.pi * t + math.pi/2)
        
        # Joint positions
        joint_state.position = [
            base_rotation,      # base_joint - rotating base
            shoulder_angle,     # shoulder_joint - shoulder movement
            elbow_angle,        # elbow_joint - elbow movement
            wrist_wave,         # wrist_1 - main waving motion
            wrist_2,            # wrist_2 - additional wrist motion
            wrist_3             # wrist_3 - twist motion
        ]
        
        # Add velocity information for smoother visualization
        joint_state.velocity = []
        joint_state.effort = []
        
        # Publish the joint states
        self.joint_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    
    # Create and run the node
    wave_demo = RobotArmWaveDemo()
    
    try:
        rclpy.spin(wave_demo)
    except KeyboardInterrupt:
        print("\n🛑 Stopping robot arm wave demo...")
    finally:
        wave_demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()