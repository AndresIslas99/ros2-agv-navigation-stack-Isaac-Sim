#!/usr/bin/env python3
# =============================================================================
# Teleop Keyboard Node
# Author: Andrés Islas Bravo
# Description: Keyboard teleoperation for AGV control during mapping/testing
# =============================================================================

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

INSTRUCTIONS = """
---------------------------
AGV Teleop Keyboard Control
---------------------------
Moving:
   w
a  s  d

w/s : forward/backward
a/d : rotate left/right
space : emergency stop

Speed controls:
q/e : increase/decrease linear speed
z/c : increase/decrease angular speed

Press CTRL+C to quit
---------------------------
"""


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Speed settings
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.linear_step = 0.05
        self.angular_step = 0.1
        
        self.get_logger().info('Teleop keyboard initialized')
        print(INSTRUCTIONS)
        print(f'Linear speed: {self.linear_speed:.2f} | Angular speed: {self.angular_speed:.2f}')
    
    def get_key(self):
        """Get single keypress from terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def run(self):
        """Main control loop"""
        try:
            while True:
                key = self.get_key()
                
                twist = Twist()
                
                if key == 'w':
                    twist.linear.x = self.linear_speed
                elif key == 's':
                    twist.linear.x = -self.linear_speed
                elif key == 'a':
                    twist.angular.z = self.angular_speed
                elif key == 'd':
                    twist.angular.z = -self.angular_speed
                elif key == ' ':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    print('\n[EMERGENCY STOP]')
                elif key == 'q':
                    self.linear_speed += self.linear_step
                    print(f'\nLinear speed: {self.linear_speed:.2f}')
                    continue
                elif key == 'e':
                    self.linear_speed = max(0.0, self.linear_speed - self.linear_step)
                    print(f'\nLinear speed: {self.linear_speed:.2f}')
                    continue
                elif key == 'z':
                    self.angular_speed += self.angular_step
                    print(f'\nAngular speed: {self.angular_speed:.2f}')
                    continue
                elif key == 'c':
                    self.angular_speed = max(0.0, self.angular_speed - self.angular_step)
                    print(f'\nAngular speed: {self.angular_speed:.2f}')
                    continue
                elif key == '\x03':  # CTRL+C
                    break
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                
                self.publisher.publish(twist)
                
        except Exception as e:
            print(f'Error: {e}')
        finally:
            twist = Twist()
            self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
