#!/usr/bin/env python3
"""
Simple gantry motion example.
Sends move commands to the gantry simulator.
"""

import rospy
from robot_movement_interface.msg import CommandList, Command

def move_to(x, y, z, velocity=0.1):
    """Send a linear move command to the gantry."""
    pub = rospy.Publisher('/command_list', CommandList, queue_size=1)
    rospy.sleep(0.5)  # Wait for publisher
    
    cmd = Command()
    cmd.command_id = 1
    cmd.command_type = 'LIN'
    cmd.pose = [x, y, z, 0, 0, 0]
    cmd.velocity = [velocity]
    cmd.velocity_type = 'M/S'
    
    cmd_list = CommandList()
    cmd_list.replace_previous_commands = True
    cmd_list.commands = [cmd]
    
    pub.publish(cmd_list)
    rospy.loginfo(f"Moving to ({x:.2f}, {y:.2f}, {z:.2f})")

def demo_sequence():
    """Run a simple demonstration sequence."""
    rospy.init_node('gantry_move_example')
    rospy.sleep(1)
    
    rospy.loginfo("Starting gantry motion demo")
    
    # Move to corner positions
    move_to(0.2, 0.15, 0.0)
    rospy.sleep(3)
    
    move_to(0.2, -0.15, 0.0)
    rospy.sleep(3)
    
    move_to(-0.2, -0.15, 0.0)
    rospy.sleep(3)
    
    move_to(-0.2, 0.15, 0.0)
    rospy.sleep(3)
    
    # Return to center
    move_to(0.0, 0.0, 0.0)
    rospy.loginfo("Demo complete")

if __name__ == '__main__':
    try:
        demo_sequence()
    except rospy.ROSInterruptException:
        pass
