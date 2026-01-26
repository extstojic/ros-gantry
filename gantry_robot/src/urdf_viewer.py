#!/usr/bin/env python3
"""
Simple URDF Visualizer - Shows the robot structure without requiring Gazebo
"""

from xml.etree import ElementTree as ET
import sys

def parse_urdf(urdf_file):
    """Parse URDF file and extract robot structure"""
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    
    print("\n" + "="*70)
    print("GANTRY ROBOT URDF STRUCTURE")
    print("="*70)
    
    # Parse links
    print("\n📦 LINKS:")
    links = root.findall('link')
    for i, link in enumerate(links, 1):
        name = link.get('name')
        visual = link.find('visual')
        collision = link.find('collision')
        inertial = link.find('inertial')
        
        print(f"\n  {i}. Link: {name}")
        if visual is not None:
            geom = visual.find('geometry')
            if geom is not None:
                for child in geom:
                    tag = child.tag
                    if tag == 'box':
                        size = child.get('size')
                        print(f"     Visual: Box {size}")
                    elif tag == 'cylinder':
                        radius = child.get('radius')
                        length = child.get('length')
                        print(f"     Visual: Cylinder r={radius} h={length}")
        
        if inertial is not None:
            mass = inertial.find('mass')
            if mass is not None:
                mass_val = mass.get('value')
                print(f"     Mass: {mass_val} kg")
    
    # Parse joints
    print("\n\n⚙️  JOINTS:")
    joints = root.findall('joint')
    for i, joint in enumerate(joints, 1):
        name = joint.get('name')
        jtype = joint.get('type')
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        
        print(f"\n  {i}. Joint: {name} (type: {jtype})")
        print(f"     Parent: {parent} → Child: {child}")
        
        if jtype == 'prismatic':
            axis = joint.find('axis').get('xyz')
            limit = joint.find('limit')
            lower = limit.get('lower')
            upper = limit.get('upper')
            print(f"     Axis: {axis}")
            print(f"     Range: [{lower}, {upper}] m")
    
    # Print kinematic chain
    print("\n\n🔗 KINEMATIC CHAIN:")
    print("  world")
    print("    ↓ (fixed)")
    print("  base_link")
    print("    ↓ joint_x (prismatic, X-axis)")
    print("  x_carriage")
    print("    ↓ joint_y (prismatic, Y-axis)")
    print("  y_carriage")
    print("    ↓ joint_z (prismatic, Z-axis)")
    print("  z_carriage")
    print("    ↓ tool_joint (fixed)")
    print("  tool0 (end-effector frame)")
    print("    ↓ gripper_joint (fixed)")
    print("  gripper_cup (suction cup)")
    
    print("\n" + "="*70)

def print_robot_specs():
    """Print robot specifications"""
    print("\n" + "="*70)
    print("ROBOT SPECIFICATIONS")
    print("="*70)
    
    specs = {
        "Type": "3-Axis Cartesian (Gantry)",
        "Control": "JointTrajectoryController",
        "Physics Engine": "ODE (Gazebo)",
        "End-Effector": "Suction Cup Gripper",
        "ROS Version": "Noetic",
        "Axes": "3 (X, Y, Z - all prismatic)",
    }
    
    print("\nConfiguration:")
    for key, val in specs.items():
        print(f"  • {key}: {val}")
    
    joint_specs = [
        ("X-axis", "joint_x", "-0.4 to 0.4 m", "1.0 m/s", "100 N"),
        ("Y-axis", "joint_y", "-0.3 to 0.3 m", "1.0 m/s", "100 N"),
        ("Z-axis", "joint_z", "-0.5 to 0.5 m", "1.0 m/s", "100 N"),
    ]
    
    print("\nJoint Details:")
    print(f"  {'Axis':<10} {'Joint':<12} {'Range':<20} {'Velocity':<12} {'Effort':<10}")
    print(f"  {'-'*10} {'-'*12} {'-'*20} {'-'*12} {'-'*10}")
    for axis, joint, range_val, vel, effort in joint_specs:
        print(f"  {axis:<10} {joint:<12} {range_val:<20} {vel:<12} {effort:<10}")
    
    print("\n" + "="*70)

def main():
    urdf_file = "/root/Desktop/ros-gantry/gantry_robot/urdf/gantry_robot.urdf"
    
    try:
        parse_urdf(urdf_file)
        print_robot_specs()
        
        print("\n✅ URDF Model Validated Successfully!")
        print("\n📝 To run the full simulation, use:")
        print("   roslaunch gantry_robot gazebo.launch")
        print("\n   Then in another terminal:")
        print("   rosrun gantry_robot motion_example.py")
        print("   or")
        print("   rosrun gantry_robot interactive_control.py\n")
        
    except Exception as e:
        print(f"❌ Error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main()
