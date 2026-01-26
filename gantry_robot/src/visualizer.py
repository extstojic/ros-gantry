#!/usr/bin/env python3
"""
Simple terminal-based robot visualizer - no Gazebo GUI needed!
Shows joint positions in real-time with ASCII graphics.
"""

import rospy
from sensor_msgs.msg import JointState
import os
import sys

class TerminalVisualizer:
    def __init__(self):
        rospy.init_node('terminal_visualizer', anonymous=True)
        self.positions = {'joint_x': 0.0, 'joint_y': 0.0, 'joint_z': 0.0}
        rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        
    def joint_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.positions:
                self.positions[name] = pos
    
    def clear_screen(self):
        os.system('clear' if os.name != 'nt' else 'cls')
    
    def draw_bar(self, value, min_val, max_val, width=40):
        normalized = (value - min_val) / (max_val - min_val)
        filled = int(normalized * width)
        filled = max(0, min(width, filled))
        return '[' + '█' * filled + '░' * (width - filled) + ']'
    
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz update
        
        while not rospy.is_shutdown():
            self.clear_screen()
            
            x = self.positions['joint_x']
            y = self.positions['joint_y']
            z = self.positions['joint_z']
            
            print("=" * 60)
            print("  GANTRY ROBOT - TERMINAL VISUALIZER")
            print("  (Run Gazebo headless: gui:=false)")
            print("=" * 60)
            print()
            print(f"  X Position: {x:+.4f} m  (range: -0.4 to +0.4)")
            print(f"  {self.draw_bar(x, -0.4, 0.4)}")
            print()
            print(f"  Y Position: {y:+.4f} m  (range: -0.3 to +0.3)")
            print(f"  {self.draw_bar(y, -0.3, 0.3)}")
            print()
            print(f"  Z Position: {z:+.4f} m  (range: -0.5 to +0.5)")
            print(f"  {self.draw_bar(z, -0.5, 0.5)}")
            print()
            print("=" * 60)
            print()
            
            # Simple ASCII side view
            z_height = int((z + 0.5) * 10)  # 0-10 scale
            x_pos = int((x + 0.4) * 20)     # 0-16 scale
            
            print("  Side View (X-Z plane):")
            print("  " + "─" * 22)
            for row in range(10, -1, -1):
                line = "  │"
                for col in range(20):
                    if row == z_height and col == x_pos:
                        line += "◆"  # End effector
                    elif row == 10:
                        line += "═"  # Top rail
                    else:
                        line += " "
                line += "│"
                print(line)
            print("  " + "─" * 22)
            print("       X axis →")
            print()
            print("  Press Ctrl+C to exit")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        TerminalVisualizer().run()
    except rospy.ROSInterruptException:
        pass
