#!/usr/bin/env python3
"""
Simple Demo - Move robot to home, lift up, return home
Gazebo starts PAUSED so joints don't drift. This script unpauses after setup.
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

class Demo:
    def __init__(self):
        rospy.init_node('demo', anonymous=True)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("DEMO NODE STARTING")
        rospy.loginfo("Gazebo should be PAUSED - joints won't drift")
        rospy.loginfo("=" * 60)
        
        # Connect to controller action servers
        self.x_client = actionlib.SimpleActionClient('/joint_x_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.y_client = actionlib.SimpleActionClient('/joint_y_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.z_client = actionlib.SimpleActionClient('/joint_z_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        rospy.loginfo("[INIT] Waiting for controller action servers...")
        self.x_client.wait_for_server()
        self.y_client.wait_for_server()
        self.z_client.wait_for_server()
        rospy.loginfo("[INIT] All controller action servers connected")
        
        # Setup unpause service
        rospy.loginfo("[INIT] Connecting to Gazebo services...")
        rospy.wait_for_service('/gazebo/unpause_physics', timeout=10.0)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        rospy.loginfo("[INIT] Ready to unpause Gazebo")
        
        rospy.loginfo("=" * 60)

    def get_current_positions(self, timeout=5.0):
        js = rospy.wait_for_message('/joint_states', JointState, timeout=timeout)
        pos = {}
        for name, p in zip(js.name, js.position):
            if name in ('joint_x', 'joint_y', 'joint_z'):
                pos[name] = p
        for j in ('joint_x', 'joint_y', 'joint_z'):
            if j not in pos:
                pos[j] = 0.0
        return pos

    def log_positions(self, label):
        try:
            pos = self.get_current_positions()
            rospy.loginfo("[DEBUG] %s: X=%.4f, Y=%.4f, Z=%.4f" % 
                         (label, pos['joint_x'], pos['joint_y'], pos['joint_z']))
        except Exception as e:
            rospy.logwarn("[DEBUG] Could not read positions for %s: %s" % (label, e))

    def move(self, x, y, z, duration=2.0):
        rospy.loginfo("-" * 40)
        rospy.loginfo("[MOVE] Target: X=%.4f, Y=%.4f, Z=%.4f (duration=%.1fs)" % (x, y, z, duration))
        self.log_positions("BEFORE move")
        
        for client, joint, pos in [(self.x_client, 'joint_x', x), 
                                    (self.y_client, 'joint_y', y), 
                                    (self.z_client, 'joint_z', z)]:
            traj = JointTrajectory()
            traj.joint_names = [joint]
            pt = JointTrajectoryPoint()
            pt.positions = [pos]
            pt.time_from_start = rospy.Duration(duration)
            traj.points = [pt]
            goal = FollowJointTrajectoryGoal(trajectory=traj)
            client.send_goal(goal)
        
        self.x_client.wait_for_result()
        self.y_client.wait_for_result()
        self.z_client.wait_for_result()
        
        self.log_positions("AFTER move")
        rospy.loginfo("-" * 40)

    def run(self):
        rospy.loginfo("=" * 60)
        rospy.loginfo("=== DEMO START ===")
        rospy.loginfo("=" * 60)
        
        # Unpause physics - simulation will start NOW
        rospy.loginfo("[RUN] Unpausing Gazebo physics...")
        self.unpause()
        rospy.loginfo("[RUN] Gazebo physics UNPAUSED - simulation running")
        
        # Small delay to let physics settle and joint_states publish
        rospy.sleep(0.2)
        self.log_positions("IMMEDIATELY AFTER UNPAUSE")

        # Step 1: Go to home (0,0,0) - this ensures we start at known position
        rospy.loginfo("")
        rospy.loginfo("[STEP 1] Go to home (0, 0, 0)")
        rospy.loginfo("EXPECTED AFTER: X=0.0000, Y=0.0000, Z=0.0000")
        self.move(0, 0, 0, duration=2.0)
        rospy.sleep(1.0)
        self.log_positions("AFTER STEP 1 + 1s settle")
        
        # Step 2: Lift up Z axis
        rospy.loginfo("")
        rospy.loginfo("[STEP 2] Lift up Z axis to 0.4")
        rospy.loginfo("EXPECTED AFTER: X=0.0000, Y=0.0000, Z=0.4000")
        self.move(0, 0, 0.4, duration=2.0)
        rospy.sleep(1.0)
        self.log_positions("AFTER STEP 2 + 1s settle")
        
        # Step 3: Return to home
        rospy.loginfo("")
        rospy.loginfo("[STEP 3] Return to home (0, 0, 0)")
        rospy.loginfo("EXPECTED AFTER: X=0.0000, Y=0.0000, Z=0.0000")
        self.move(0, 0, 0, duration=2.0)
        rospy.sleep(1.0)
        self.log_positions("AFTER STEP 3 + 1s settle")
        
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("=== DEMO COMPLETE ===")
        self.log_positions("FINAL POSITIONS")
        rospy.loginfo("=" * 60)

if __name__ == '__main__':
    Demo().run()
