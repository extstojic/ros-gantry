#!/usr/bin/env python3
"""
Marker Initialization Synchronization Script

This script ensures that the interactive marker is initialized with the correct
robot position rather than [0, 0, 0]. It waits for the gantry simulator to 
publish its first tool frame position, then restarts the interactive marker.
"""

import rospy
import subprocess
import time
from robot_movement_interface.msg import EulerFrame

def wait_for_tool_frame():
    """Wait for the first tool frame message"""
    rospy.loginfo("Waiting for tool frame to be published...")
    try:
        msg = rospy.wait_for_message(
            '/dnb_gantry_simulator/tool_frame', 
            EulerFrame, 
            timeout=10.0
        )
        rospy.loginfo(f"Tool frame received: x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}")
        return True
    except rospy.ROSException as e:
        rospy.logerr(f"Timeout waiting for tool frame: {e}")
        return False

def restart_interactive_marker():
    """Restart the interactive marker node to pick up correct position"""
    rospy.loginfo("Restarting interactive marker with correct initialization...")
    try:
        # Kill any existing marker processes
        subprocess.run(['pkill', '-9', '-f', 'interactive_marker'], 
                      capture_output=True, check=False)
        time.sleep(1)
        
        # Start the marker via roslaunch
        subprocess.Popen([
            'roslaunch', 
            'dnb_remote_robot_control', 
            'start_v3.launch',
            'package:=dnb_gantry_simulator',
            'robot:=gantry'
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        rospy.loginfo("Interactive marker restarted successfully")
        return True
    except Exception as e:
        rospy.logerr(f"Error restarting interactive marker: {e}")
        return False

def main():
    rospy.init_node('marker_init_sync', anonymous=True)
    
    rospy.loginfo("Marker initialization sync node started")
    
    # Wait for tool frame to be published
    if wait_for_tool_frame():
        time.sleep(1)  # Give it a moment to settle
        restart_interactive_marker()
    else:
        rospy.logwarn("Could not wait for tool frame - marker may initialize at [0,0,0]")
    
    rospy.loginfo("Marker initialization sync complete")
    rospy.spin()

if __name__ == '__main__':
    main()
