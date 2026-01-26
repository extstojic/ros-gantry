#!/usr/bin/env python3
"""
ASCII 3D Visualizer for Gantry Robot - Shows robot motion in terminal
Demonstrates the robot moving through pick-and-place positions
"""

import time
import os

def clear_screen():
    """Clear terminal screen"""
    os.system('clear' if os.name == 'posix' else 'cls')

def draw_grid(x_pos, y_pos, z_pos):
    """Draw 3D grid representation of robot position"""
    # Scale positions to grid (0-20 range)
    grid_x = int((x_pos + 0.4) * 25)  # X: -0.4 to 0.4
    grid_y = int((y_pos + 0.3) * 33)  # Y: -0.3 to 0.3
    grid_z = int((z_pos + 0.5) * 20)  # Z: -0.5 to 0.5
    
    # Clamp to grid bounds
    grid_x = max(0, min(20, grid_x))
    grid_y = max(0, min(20, grid_y))
    grid_z = max(0, min(20, grid_z))
    
    print("\n" + "="*70)
    print("3D ROBOT VISUALIZATION")
    print("="*70)
    
    print("\n  Z-AXIS VIEW (from Y):")
    print("  " + "+"*22)
    
    for z in range(20, -1, -1):
        line = "  "
        for x in range(21):
            if x == grid_x and z == grid_z:
                line += "🤖"
            elif z == 0:
                line += "─"
            elif x == 0 or x == 20:
                line += "│"
            else:
                line += " "
        
        if z % 5 == 0:
            line += f"  Z={z/10:.1f}m"
        print(line)
    
    print("  " + "+"*22)
    print("  X=" + " "*8 + "X=0.4m" + " "*7)
    
    print("\n  Y-AXIS VIEW (from X):")
    print("  " + "+"*22)
    
    for z in range(20, -1, -1):
        line = "  "
        for y in range(21):
            if y == grid_y and z == grid_z:
                line += "🤖"
            elif z == 0:
                line += "─"
            elif y == 0 or y == 20:
                line += "│"
            else:
                line += " "
        
        if z % 5 == 0:
            line += f"  Z={z/10:.1f}m"
        print(line)
    
    print("  " + "+"*22)
    print("  Y=" + " "*8 + "Y=0.3m" + " "*7)
    
    print("\n  X-Y TOP VIEW:")
    print("  " + "+"*22)
    
    for y in range(20, -1, -1):
        line = "  "
        for x in range(21):
            if x == grid_x and y == grid_y:
                line += "●"
            elif x == 0 or x == 20 or y == 0 or y == 20:
                line += "+"
            else:
                line += " "
        
        if y % 5 == 0:
            line += f"  Y={y/10:.1f}m"
        print(line)
    
    print("  " + "+"*22)
    print("  X=" + " "*8 + "X=0.4m" + " "*7)

def show_position(x, y, z, label, duration=2):
    """Display robot at specific position"""
    clear_screen()
    
    print("\n" + "╔" + "="*68 + "╗")
    print(f"║ {label:<66} ║")
    print("╚" + "="*68 + "╝")
    
    draw_grid(x, y, z)
    
    print(f"\n  Position: X={x:6.3f}m  Y={y:6.3f}m  Z={z:6.3f}m")
    print(f"\n  [Moving for {duration} seconds...]")
    
    time.sleep(duration)

def run_visualization():
    """Run motion visualization sequence"""
    
    positions = [
        (0.0, 0.0, 0.0, "HOME POSITION - Ready State"),
        (0.2, 0.0, 0.0, "MOVING RIGHT (X-axis)"),
        (0.2, 0.1, 0.0, "MOVING FORWARD (Y-axis)"),
        (0.2, 0.1, -0.3, "MOVING DOWN (Z-axis) - Pick Position"),
        (0.2, 0.1, 0.2, "LIFTING OBJECT"),
        (-0.2, 0.1, 0.2, "MOVING LEFT (X-axis)"),
        (-0.2, 0.2, 0.2, "MOVING FORWARD (Y-axis)"),
        (-0.2, 0.2, -0.2, "LOWERING (Z-axis) - Place Position"),
        (-0.2, 0.2, 0.2, "LIFTING GRIPPER"),
        (0.0, 0.0, 0.0, "RETURNING TO HOME"),
    ]
    
    print("\n" + "="*70)
    print("GANTRY ROBOT - ASCII 3D VISUALIZATION")
    print("="*70)
    print("\nShowing pick-and-place motion sequence...\n")
    time.sleep(2)
    
    for x, y, z, label in positions:
        show_position(x, y, z, label, duration=2)
    
    clear_screen()
    print("\n" + "="*70)
    print("✅ VISUALIZATION COMPLETE")
    print("="*70)
    print("\nThis is an ASCII visualization of the robot's motion.")
    print("\nTo see the full Gazebo simulation:")
    print("  1. Open Terminal 1: roslaunch gantry_robot gazebo.launch")
    print("  2. Open Terminal 2: rosrun gantry_robot motion_example.py")
    print("\n" + "="*70 + "\n")

if __name__ == '__main__':
    try:
        run_visualization()
    except KeyboardInterrupt:
        print("\n\nVisualization interrupted.")
