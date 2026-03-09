#!/usr/bin/env python3
"""
Pick and place example for gantry robot.
Demonstrates a simple pick-and-place sequence.
"""

import rospy
from robot_movement_interface.msg import CommandList, Command

class PickAndPlace:
    def __init__(self):
        self.pub = rospy.Publisher('/command_list', CommandList, queue_size=1)
        rospy.sleep(1)  # Wait for publisher
    
    def move_to(self, x, y, z, velocity=0.1, wait=True):
        """Send move command to gantry."""
        cmd = Command()
        cmd.command_id = 1
        cmd.command_type = 'LIN'
        cmd.pose = [x, y, z, 0, 0, 0]
        cmd.velocity = [velocity]
        cmd.velocity_type = 'M/S'
        
        cmd_list = CommandList()
        cmd_list.replace_previous_commands = True
        cmd_list.commands = [cmd]
        
        self.pub.publish(cmd_list)
        rospy.loginfo(f"Moving to ({x:.2f}, {y:.2f}, {z:.2f})")
        
        if wait:
            rospy.sleep(3)
    
    def pick_and_place(self, pick_pos, place_pos, approach_height=0.1):
        """Execute pick and place operation."""
        px, py = pick_pos
        plx, ply = place_pos
        
        # Approach pick position
        rospy.loginfo("Approaching pick position")
        self.move_to(px, py, approach_height)
        
        # Lower to pick
        rospy.loginfo("Picking")
        self.move_to(px, py, 0.0)
        rospy.sleep(0.5)
        
        # Lift object
        rospy.loginfo("Lifting")
        self.move_to(px, py, approach_height)
        
        # Move to place position
        rospy.loginfo("Moving to place position")
        self.move_to(plx, ply, approach_height)
        
        # Lower to place
        rospy.loginfo("Placing")
        self.move_to(plx, ply, 0.0)
        rospy.sleep(0.5)
        
        # Retract
        rospy.loginfo("Retracting")
        self.move_to(plx, ply, approach_height)
        
        # Return home
        rospy.loginfo("Returning home")
        self.move_to(0.0, 0.0, 0.0)

def run_demo():
    """Run pick and place demo."""
    rospy.init_node('pick_and_place_example')
    
    pp = PickAndPlace()
    
    rospy.loginfo("Starting pick and place demo")
    
    # Pick from right, place on left
    pp.pick_and_place(
        pick_pos=(0.2, 0.1),
        place_pos=(-0.2, -0.1)
    )
    
    rospy.loginfo("Demo complete")

if __name__ == '__main__':
    try:
        run_demo()
    except rospy.ROSInterruptException:
        pass
