#!/usr/bin/env python3
"""
Interactive control interface for the Gantry robot.
Allows manual control of robot position via command line.
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

class InteractiveGantryControl:
    def __init__(self):
        """Initialize interactive control."""
        rospy.init_node('interactive_gantry_control', anonymous=True)
        
        # Create action clients
        self.x_client = actionlib.SimpleActionClient(
            '/joint_x_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        self.y_client = actionlib.SimpleActionClient(
            '/joint_y_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        self.z_client = actionlib.SimpleActionClient(
            '/joint_z_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        # Wait for servers
        try:
            self.x_client.wait_for_server(timeout=rospy.Duration(3))
            self.y_client.wait_for_server(timeout=rospy.Duration(3))
            self.z_client.wait_for_server(timeout=rospy.Duration(3))
            self.servers_ready = True
        except rospy.ROSException:
            rospy.logwarn("Action servers not available. Waiting for simulation...")
            self.servers_ready = False
    
    def _send_trajectory(self, client, joint_name, position, duration=2.0):
        """Send trajectory to a single joint."""
        if not self.servers_ready:
            rospy.logerr("Servers not ready!")
            return False
        
        trajectory = JointTrajectory()
        trajectory.joint_names = [joint_name]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = rospy.Duration(duration)
        trajectory.points.append(point)
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        client.send_goal(goal)
        return True
    
    def move(self, x=None, y=None, z=None, duration=2.0):
        """Move to position (only specified axes)."""
        if x is not None:
            x = max(-0.4, min(0.4, x))
            self._send_trajectory(self.x_client, 'joint_x', x, duration)
        
        if y is not None:
            y = max(-0.3, min(0.3, y))
            self._send_trajectory(self.y_client, 'joint_y', y, duration)
        
        if z is not None:
            z = max(-0.5, min(0.5, z))
            self._send_trajectory(self.z_client, 'joint_z', z, duration)
    
    def wait_completion(self):
        """Wait for all movements to complete."""
        if self.servers_ready:
            self.x_client.wait_for_result()
            self.y_client.wait_for_result()
            self.z_client.wait_for_result()
    
    def home(self):
        """Return to home position."""
        rospy.loginfo("Moving to home position...")
        self.move(x=0, y=0, z=0, duration=2.0)
        self.wait_completion()
    
    def show_help(self):
        """Display help message."""
        help_text = """
╔════════════════════════════════════════════════════════════╗
║        GANTRY ROBOT INTERACTIVE CONTROL                    ║
╚════════════════════════════════════════════════════════════╝

Commands:
  move <x> <y> <z>  - Move to position (values in meters)
                       Range: X[-0.4,0.4] Y[-0.3,0.3] Z[-0.5,0.5]
  
  mv <x> <y> <z>    - Shorthand for move
  
  x <pos>            - Move only X axis
  y <pos>            - Move only Y axis
  z <pos>            - Move only Z axis
  
  home               - Return to home (0,0,0)
  
  limits             - Show joint limits
  
  demo               - Run motion demonstration
  pick               - Run pick and place demo
  
  help               - Show this help
  quit/exit          - Exit program

Examples:
  > move 0.2 0.1 -0.3    # Move to X=0.2, Y=0.1, Z=-0.3
  > x 0.3                # Move X axis to 0.3
  > z -0.2               # Move Z axis to -0.2
  > home                 # Return to origin
  > demo                 # Run demo sequence

        """
        print(help_text)
    
    def run_interactive(self):
        """Run interactive command loop."""
        if not self.servers_ready:
            rospy.logerr("Cannot start: ROS action servers not available")
            rospy.logerr("Make sure to run: roslaunch gantry_robot gazebo.launch")
            return
        
        print("\n" + "="*60)
        print("GANTRY ROBOT INTERACTIVE CONTROL")
        print("="*60)
        print("Type 'help' for commands or 'quit' to exit\n")
        
        try:
            while not rospy.is_shutdown():
                try:
                    user_input = input("gantry> ").strip().lower()
                    
                    if not user_input:
                        continue
                    
                    parts = user_input.split()
                    cmd = parts[0]
                    
                    # Help command
                    if cmd == 'help':
                        self.show_help()
                    
                    # Quit command
                    elif cmd in ['quit', 'exit', 'q']:
                        rospy.loginfo("Exiting...")
                        break
                    
                    # Home command
                    elif cmd == 'home':
                        self.home()
                    
                    # Limits display
                    elif cmd == 'limits':
                        print("\nJoint Limits:")
                        print("  X (joint_x): -0.4 to 0.4 m")
                        print("  Y (joint_y): -0.3 to 0.3 m")
                        print("  Z (joint_z): -0.5 to 0.5 m\n")
                    
                    # Demo command
                    elif cmd == 'demo':
                        self.run_demo()
                    
                    # Pick and place demo
                    elif cmd == 'pick':
                        self.run_pick_demo()
                    
                    # Move commands
                    elif cmd in ['move', 'mv']:
                        if len(parts) >= 4:
                            try:
                                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                                self.move(x=x, y=y, z=z, duration=2.0)
                                self.wait_completion()
                                rospy.loginfo(f"Reached position ({x}, {y}, {z})")
                            except ValueError:
                                rospy.logerr("Invalid values. Use: move <x> <y> <z>")
                        else:
                            rospy.logerr("Usage: move <x> <y> <z>")
                    
                    # Single axis commands
                    elif cmd == 'x':
                        if len(parts) >= 2:
                            try:
                                x = float(parts[1])
                                self.move(x=x, duration=2.0)
                                self.wait_completion()
                                rospy.loginfo(f"X moved to {x}")
                            except ValueError:
                                rospy.logerr("Invalid value")
                        else:
                            rospy.logerr("Usage: x <position>")
                    
                    elif cmd == 'y':
                        if len(parts) >= 2:
                            try:
                                y = float(parts[1])
                                self.move(y=y, duration=2.0)
                                self.wait_completion()
                                rospy.loginfo(f"Y moved to {y}")
                            except ValueError:
                                rospy.logerr("Invalid value")
                        else:
                            rospy.logerr("Usage: y <position>")
                    
                    elif cmd == 'z':
                        if len(parts) >= 2:
                            try:
                                z = float(parts[1])
                                self.move(z=z, duration=2.0)
                                self.wait_completion()
                                rospy.loginfo(f"Z moved to {z}")
                            except ValueError:
                                rospy.logerr("Invalid value")
                        else:
                            rospy.logerr("Usage: z <position>")
                    
                    else:
                        rospy.logwarn(f"Unknown command: {cmd}. Type 'help' for available commands.")
                
                except KeyboardInterrupt:
                    rospy.loginfo("\nInterrupted by user")
                    break
        
        except EOFError:
            rospy.loginfo("EOF reached, exiting...")
    
    def run_demo(self):
        """Run motion demonstration."""
        rospy.loginfo("Starting motion demonstration...")
        positions = [
            (0, 0, 0),
            (0.3, 0, 0),
            (0.3, 0.2, 0),
            (0.3, 0.2, -0.3),
            (0.3, -0.2, -0.3),
            (-0.3, -0.2, -0.3),
            (-0.3, -0.2, 0.2),
            (0, 0, 0),
        ]
        
        for i, (x, y, z) in enumerate(positions):
            rospy.loginfo(f"Position {i+1}/{len(positions)}: ({x}, {y}, {z})")
            self.move(x=x, y=y, z=z, duration=2.0)
            self.wait_completion()
            rospy.sleep(0.5)
        
        rospy.loginfo("Demo complete!")
    
    def run_pick_demo(self):
        """Run pick and place demonstration."""
        rospy.loginfo("Starting pick and place demo...")
        
        # Home
        self.move(x=0, y=0, z=0, duration=2.0)
        self.wait_completion()
        rospy.sleep(1)
        
        # Approach pick position
        self.move(x=0.2, y=0.1, z=0.3, duration=2.0)
        self.wait_completion()
        rospy.sleep(0.5)
        
        # Pick down
        self.move(x=0.2, y=0.1, z=-0.2, duration=1.5)
        self.wait_completion()
        rospy.loginfo("[GRIPPER] Closing...")
        rospy.sleep(1)
        
        # Lift
        self.move(x=0.2, y=0.1, z=0.3, duration=1.5)
        self.wait_completion()
        rospy.sleep(0.5)
        
        # Move to place location
        self.move(x=-0.2, y=0.2, z=0.3, duration=2.0)
        self.wait_completion()
        rospy.sleep(0.5)
        
        # Place down
        self.move(x=-0.2, y=0.2, z=-0.2, duration=1.5)
        self.wait_completion()
        rospy.loginfo("[GRIPPER] Opening...")
        rospy.sleep(1)
        
        # Retract
        self.move(x=-0.2, y=0.2, z=0.3, duration=1.5)
        self.wait_completion()
        rospy.sleep(0.5)
        
        # Home
        self.move(x=0, y=0, z=0, duration=2.0)
        self.wait_completion()
        
        rospy.loginfo("Pick and place demo complete!")

def main():
    try:
        controller = InteractiveGantryControl()
        controller.run_interactive()
    except rospy.ROSInterruptException:
        rospy.loginfo("Control interface interrupted")

if __name__ == '__main__':
    main()
