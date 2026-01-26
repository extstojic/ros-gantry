#!/usr/bin/env python3
"""
Interactive Robot Simulator - Shows pick-and-place task with real-time visualization
No Gazebo or ROS required - pure Python simulation
"""

import time
import sys
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class Position:
    """Robot position in 3D space"""
    x: float
    y: float
    z: float
    
    def __str__(self):
        return f"({self.x:6.3f}, {self.y:6.3f}, {self.z:6.3f})"

class GantryRobotSimulator:
    """Simulates 3-axis gantry robot motion"""
    
    def __init__(self):
        self.position = Position(0.0, 0.0, 0.0)
        self.gripper_closed = False
        self.max_velocity = 1.0  # m/s
        self.step_count = 0
        
    def move_to(self, target: Position, duration: float = 2.0):
        """Move robot to target position over duration seconds"""
        start = self.position
        steps = int(duration * 10)  # 100ms steps
        
        for step in range(steps + 1):
            t = step / steps if steps > 0 else 1.0
            
            # Linear interpolation
            self.position = Position(
                x=start.x + (target.x - start.x) * t,
                y=start.y + (target.y - start.y) * t,
                z=start.z + (target.z - start.z) * t,
            )
            
            self.step_count += 1
            self.render()
            time.sleep(0.05)
    
    def render(self):
        """Render current robot state"""
        # Clear and show position
        sys.stdout.write(f"\r Position: {self.position} | Gripper: {'CLOSED' if self.gripper_closed else 'OPEN '}")
        sys.stdout.flush()
    
    def gripper_open(self):
        """Open gripper"""
        self.gripper_closed = False
        sys.stdout.write(f"\r ✓ Gripper OPEN  | Position: {self.position}\n")
        sys.stdout.flush()
        time.sleep(0.5)
    
    def gripper_close(self):
        """Close gripper"""
        self.gripper_closed = True
        sys.stdout.write(f"\r ✓ Gripper CLOSED | Position: {self.position}\n")
        sys.stdout.flush()
        time.sleep(0.5)

def print_header(title):
    """Print section header"""
    print("\n" + "="*70)
    print(f"  {title}")
    print("="*70)

def run_demo():
    """Run complete pick-and-place demonstration"""
    
    robot = GantryRobotSimulator()
    
    print_header("GANTRY ROBOT - INTERACTIVE SIMULATION")
    print("\nThis is a software simulation of the 3-axis Cartesian robot.")
    print("Watch the robot perform a pick-and-place assembly task.\n")
    time.sleep(1)
    
    # Task 1: Pick object from source
    print_header("TASK 1: PICK OBJECT FROM SOURCE")
    
    print("\nStep 1: Move to approach position (0.2, 0.1, 0.3)")
    robot.move_to(Position(0.2, 0.1, 0.3), duration=1.5)
    
    print("\nStep 2: Move down to pick position (0.2, 0.1, -0.2)")
    robot.move_to(Position(0.2, 0.1, -0.2), duration=1.0)
    
    print("\nStep 3: Close gripper to grip object")
    robot.gripper_close()
    
    print("\nStep 4: Lift object (0.2, 0.1, 0.3)")
    robot.move_to(Position(0.2, 0.1, 0.3), duration=1.0)
    
    # Task 2: Place object at destination
    print_header("TASK 2: PLACE OBJECT AT DESTINATION")
    
    print("\nStep 1: Move to destination X position (-0.2, 0.1, 0.3)")
    robot.move_to(Position(-0.2, 0.1, 0.3), duration=1.5)
    
    print("\nStep 2: Move to destination Y position (-0.2, 0.2, 0.3)")
    robot.move_to(Position(-0.2, 0.2, 0.3), duration=1.0)
    
    print("\nStep 3: Lower to place position (-0.2, 0.2, -0.2)")
    robot.move_to(Position(-0.2, 0.2, -0.2), duration=1.0)
    
    print("\nStep 4: Open gripper to release object")
    robot.gripper_open()
    
    print("\nStep 5: Retract gripper (-0.2, 0.2, 0.3)")
    robot.move_to(Position(-0.2, 0.2, 0.3), duration=1.0)
    
    # Task 3: Return to home
    print_header("TASK 3: RETURN TO HOME")
    
    print("\nMoving to home position (0.0, 0.0, 0.0)")
    robot.move_to(Position(0.0, 0.0, 0.0), duration=2.0)
    
    # Summary
    print_header("SIMULATION COMPLETE")
    
    print(f"\n✅ Assembly task completed successfully!")
    print(f"\nSimulation Statistics:")
    print(f"  • Total steps: {robot.step_count}")
    print(f"  • Final position: {robot.position}")
    print(f"  • Gripper state: {'CLOSED' if robot.gripper_closed else 'OPEN'}")
    
    print("\n📝 Next Steps:")
    print("  To run the full Gazebo simulation with physics:")
    print("  1. Terminal 1: roslaunch gantry_robot gazebo.launch")
    print("  2. Terminal 2: rosrun gantry_robot motion_example.py")
    print("\n" + "="*70 + "\n")

if __name__ == '__main__':
    try:
        run_demo()
    except KeyboardInterrupt:
        print("\n\nSimulation interrupted by user.")
        sys.exit(0)
