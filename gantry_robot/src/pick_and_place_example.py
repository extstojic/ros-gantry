#!/usr/bin/env python3
"""
Pick and Place example for the Gantry robot.
This script demonstrates assembly task using pick-and-place operations.
"""

import rospy
import actionlib
import time
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
import sys

class PickAndPlaceController:
    def __init__(self):
        """Initialize the pick and place controller."""
        rospy.init_node('pick_and_place_controller', anonymous=True)
        
        # Create action clients for each axis
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
        
        # Wait for action servers
        rospy.loginfo("Waiting for action servers...")
        self.x_client.wait_for_server(timeout=rospy.Duration(5))
        self.y_client.wait_for_server(timeout=rospy.Duration(5))
        self.z_client.wait_for_server(timeout=rospy.Duration(5))
        rospy.loginfo("Action servers ready!")
        
        # Gripper state (simple simulation)
        self.gripper_closed = False
    
    def _move_joint(self, client, joint_name, position, duration=2.0):
        """Move a single joint."""
        trajectory = JointTrajectory()
        trajectory.joint_names = [joint_name]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = rospy.Duration(duration)
        trajectory.points.append(point)
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        client.send_goal(goal)
    
    def move_to_pose(self, x, y, z, duration=2.0):
        """Move to XYZ position."""
        # Clamp to limits
        x = max(-0.4, min(0.4, x))
        y = max(-0.3, min(0.3, y))
        z = max(-0.5, min(0.5, z))
        
        rospy.loginfo(f"Moving to ({x:.3f}, {y:.3f}, {z:.3f})")
        
        self._move_joint(self.x_client, 'joint_x', x, duration)
        self._move_joint(self.y_client, 'joint_y', y, duration)
        self._move_joint(self.z_client, 'joint_z', z, duration)
        
        # Wait for completion
        self.x_client.wait_for_result()
        self.y_client.wait_for_result()
        self.z_client.wait_for_result()
    
    def gripper_close(self):
        """Simulate closing the gripper."""
        rospy.loginfo("Closing gripper...")
        self.gripper_closed = True
        rospy.sleep(0.5)  # Simulate gripper closing time
        rospy.loginfo("Gripper closed")
    
    def gripper_open(self):
        """Simulate opening the gripper."""
        rospy.loginfo("Opening gripper...")
        self.gripper_closed = False
        rospy.sleep(0.5)  # Simulate gripper opening time
        rospy.loginfo("Gripper opened")
    
    def pick(self, x, y, z_approach=0.3, z_pick=-0.2):
        """
        Pick an object at the specified location.
        
        Args:
            x, y: XY position of object
            z_approach: Z height to approach from
            z_pick: Z height to pick at
        """
        rospy.loginfo(f"\n=== PICKING object at ({x}, {y}) ===")
        
        # Move to approach position
        rospy.loginfo("1. Moving to approach position...")
        self.move_to_pose(x, y, z_approach, duration=2.0)
        rospy.sleep(0.5)
        
        # Move down to pick position
        rospy.loginfo("2. Moving down to pick position...")
        self.move_to_pose(x, y, z_pick, duration=1.5)
        rospy.sleep(0.5)
        
        # Close gripper
        rospy.loginfo("3. Closing gripper...")
        self.gripper_close()
        rospy.sleep(0.5)
        
        # Move up
        rospy.loginfo("4. Lifting object...")
        self.move_to_pose(x, y, z_approach, duration=1.5)
        rospy.sleep(0.5)
        
        rospy.loginfo("Pick complete!")
    
    def place(self, x, y, z_approach=0.3, z_place=-0.2):
        """
        Place an object at the specified location.
        
        Args:
            x, y: XY position to place at
            z_approach: Z height to approach from
            z_place: Z height to place at
        """
        rospy.loginfo(f"\n=== PLACING object at ({x}, {y}) ===")
        
        # Move to approach position
        rospy.loginfo("1. Moving to approach position...")
        self.move_to_pose(x, y, z_approach, duration=2.0)
        rospy.sleep(0.5)
        
        # Move down to place position
        rospy.loginfo("2. Moving down to place position...")
        self.move_to_pose(x, y, z_place, duration=1.5)
        rospy.sleep(0.5)
        
        # Open gripper
        rospy.loginfo("3. Opening gripper...")
        self.gripper_open()
        rospy.sleep(0.5)
        
        # Move up
        rospy.loginfo("4. Retracting...")
        self.move_to_pose(x, y, z_approach, duration=1.5)
        rospy.sleep(0.5)
        
        rospy.loginfo("Place complete!")
    
    def pick_and_place(self, pick_x, pick_y, place_x, place_y):
        """Complete pick and place cycle."""
        rospy.loginfo(f"\n{'='*50}")
        rospy.loginfo(f"PICK AND PLACE TASK")
        rospy.loginfo(f"Pick from: ({pick_x}, {pick_y})")
        rospy.loginfo(f"Place to: ({place_x}, {place_y})")
        rospy.loginfo(f"{'='*50}\n")
        
        # Home position
        rospy.loginfo("Moving to home position...")
        self.move_to_pose(0, 0, 0, duration=2.0)
        rospy.sleep(1.0)
        
        # Pick phase
        self.pick(pick_x, pick_y, z_approach=0.3, z_pick=-0.2)
        rospy.sleep(1.0)
        
        # Place phase
        self.place(place_x, place_y, z_approach=0.3, z_place=-0.2)
        rospy.sleep(1.0)
        
        # Return home
        rospy.loginfo("\nReturning to home position...")
        self.move_to_pose(0, 0, 0, duration=2.0)
        
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("TASK COMPLETE!")
        rospy.loginfo("="*50)
    
    def demo_assembly_task(self):
        """Demonstrate a simple assembly task with multiple pick and places."""
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("ASSEMBLY TASK DEMONSTRATION")
        rospy.loginfo("="*50 + "\n")
        
        # Task 1: Move object 1 from source to destination 1
        self.pick_and_place(pick_x=0.2, pick_y=0.1, 
                           place_x=-0.2, place_y=0.2)
        rospy.sleep(2.0)
        
        # Task 2: Move object 2 from source to destination 2
        self.pick_and_place(pick_x=-0.2, pick_y=0.1, 
                           place_x=0.2, place_y=-0.2)
        rospy.sleep(2.0)
        
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("ALL ASSEMBLY TASKS COMPLETED!")
        rospy.loginfo("="*50)

def main():
    try:
        controller = PickAndPlaceController()
        controller.demo_assembly_task()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pick and place controller interrupted")
        sys.exit(0)

if __name__ == '__main__':
    main()
