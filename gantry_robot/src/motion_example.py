#!/usr/bin/env python3
"""
Motion control example for the Gantry robot.
This script demonstrates how to move the robot using ROS action interface.
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

class GantryMotionController:
    def __init__(self):
        """Initialize the motion controller."""
        rospy.init_node('gantry_motion_controller', anonymous=True)
        
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
        
        # Wait for action servers to come up
        rospy.loginfo("Waiting for action servers...")
        self.x_client.wait_for_server(timeout=rospy.Duration(5))
        self.y_client.wait_for_server(timeout=rospy.Duration(5))
        self.z_client.wait_for_server(timeout=rospy.Duration(5))
        rospy.loginfo("Action servers ready!")
    
    def move_xyz(self, x_pos, y_pos, z_pos, duration=3.0):
        """
        Move robot to specified XYZ position.
        
        Args:
            x_pos: Target X position (m)
            y_pos: Target Y position (m)
            z_pos: Target Z position (m)
            duration: Time to reach target (seconds)
        """
        # Clamp values to joint limits
        x_pos = max(-0.4, min(0.4, x_pos))
        y_pos = max(-0.3, min(0.3, y_pos))
        z_pos = max(-0.5, min(0.5, z_pos))
        
        rospy.loginfo(f"Moving to X={x_pos:.3f}, Y={y_pos:.3f}, Z={z_pos:.3f}")
        
        # Move X axis
        trajectory_x = JointTrajectory()
        trajectory_x.joint_names = ['joint_x']
        point_x = JointTrajectoryPoint()
        point_x.positions = [x_pos]
        point_x.time_from_start = rospy.Duration(duration)
        trajectory_x.points.append(point_x)
        
        goal_x = FollowJointTrajectoryGoal()
        goal_x.trajectory = trajectory_x
        self.x_client.send_goal(goal_x)
        
        # Move Y axis
        trajectory_y = JointTrajectory()
        trajectory_y.joint_names = ['joint_y']
        point_y = JointTrajectoryPoint()
        point_y.positions = [y_pos]
        point_y.time_from_start = rospy.Duration(duration)
        trajectory_y.points.append(point_y)
        
        goal_y = FollowJointTrajectoryGoal()
        goal_y.trajectory = trajectory_y
        self.y_client.send_goal(goal_y)
        
        # Move Z axis
        trajectory_z = JointTrajectory()
        trajectory_z.joint_names = ['joint_z']
        point_z = JointTrajectoryPoint()
        point_z.positions = [z_pos]
        point_z.time_from_start = rospy.Duration(duration)
        trajectory_z.points.append(point_z)
        
        goal_z = FollowJointTrajectoryGoal()
        goal_z.trajectory = trajectory_z
        self.z_client.send_goal(goal_z)
        
        # Wait for all axes to complete
        self.x_client.wait_for_result()
        self.y_client.wait_for_result()
        self.z_client.wait_for_result()
        
        rospy.loginfo("Movement complete!")
    
    def demo_motion(self):
        """Run a demonstration motion sequence."""
        rospy.loginfo("Starting motion demonstration...")
        
        # Home position
        self.move_xyz(0, 0, 0, duration=2.0)
        rospy.sleep(1)
        
        # Move right (X+)
        self.move_xyz(0.3, 0, 0, duration=2.0)
        rospy.sleep(1)
        
        # Move forward (Y+)
        self.move_xyz(0.3, 0.2, 0, duration=2.0)
        rospy.sleep(1)
        
        # Move down (Z-)
        self.move_xyz(0.3, 0.2, -0.3, duration=2.0)
        rospy.sleep(1)
        
        # Move back (Y-)
        self.move_xyz(0.3, -0.2, -0.3, duration=2.0)
        rospy.sleep(1)
        
        # Move left (X-)
        self.move_xyz(-0.3, -0.2, -0.3, duration=2.0)
        rospy.sleep(1)
        
        # Move up (Z+)
        self.move_xyz(-0.3, -0.2, 0.2, duration=2.0)
        rospy.sleep(1)
        
        # Return to home
        self.move_xyz(0, 0, 0, duration=2.0)
        rospy.sleep(1)
        
        rospy.loginfo("Motion demonstration complete!")

def main():
    try:
        controller = GantryMotionController()
        controller.demo_motion()
    except rospy.ROSInterruptException:
        rospy.loginfo("Motion controller interrupted")
        sys.exit(0)

if __name__ == '__main__':
    main()
