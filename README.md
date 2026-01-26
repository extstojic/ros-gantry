# ROS Gantry Robot Simulation

A ROS simulation environment for a 3-axis Cartesian (Gantry) robot designed for assembly and pick-and-place tasks.

## Overview

This package provides:
- **3-axis Cartesian robot model** (X, Y, Z linear motion)
- **Pick-and-place end-effector** with gripper simulation
- **Gazebo physics simulation** with realistic workspace
- **ROS controllers** for trajectory control
- **Example scripts** for motion control and assembly tasks

## Features

✅ **3-Axis Linear Motion**: Smooth movement along X, Y, and Z axes  
✅ **Pick-and-Place Operations**: Gripper simulation for object manipulation  
✅ **Gazebo Integration**: Full physics simulation with collision detection  
✅ **ROS Control**: Standard joint trajectory controller interface  
✅ **Motion Examples**: Pre-built demos for common tasks  
✅ **Workspace Boundaries**: Configurable joint limits

## System Requirements

- **ROS**: Noetic (Ubuntu 20.04)
- **Gazebo**: 11.x or higher
- **Python**: 3.6+
- **Catkin**: Build system

## Installation

### 1. Install Dependencies

```bash
sudo apt-get update
sudo apt-get install ros-noetic-desktop-full
sudo apt-get install ros-noetic-gazebo-ros
sudo apt-get install ros-noetic-gazebo-plugins
sudo apt-get install ros-noetic-controller-manager
sudo apt-get install ros-noetic-joint-state-controller
sudo apt-get install ros-noetic-joint-trajectory-controller
sudo apt-get install ros-noetic-effort-controllers
sudo apt-get install ros-noetic-robot-state-publisher
sudo apt-get install ros-noetic-joint-state-publisher
```

### 2. Create Catkin Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. Clone/Copy Package

```bash
cp -r ros-gantry/gantry_robot ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
```

### 4. Source Setup

```bash
source ~/catkin_ws/devel/setup.bash
```

Add to `~/.bashrc` for permanent setup:
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## Package Structure

```
gantry_robot/
├── CMakeLists.txt           # Build configuration
├── package.xml              # Package metadata
├── urdf/
│   └── gantry_robot.urdf   # Robot model definition
├── launch/
│   └── gazebo.launch       # Gazebo simulation launcher
├── config/
│   └── controllers.yaml    # Joint controller configuration
├── worlds/
│   └── gantry_workspace.world  # Gazebo world with objects
├── src/
│   ├── motion_example.py        # Basic motion control demo
│   └── pick_and_place_example.py # Pick-and-place task demo
└── README.md               # This file
```

## Quick Start

### 1. Launch the Simulation

```bash
roslaunch gantry_robot gazebo.launch
```

This will:
- Start Gazebo with physics engine
- Spawn the gantry robot in the simulation
- Load controllers for XYZ axes
- Create work surface with sample objects

### 2. Run Motion Example (in another terminal)

```bash
rosrun gantry_robot motion_example.py
```

The robot will perform a demonstration motion sequence showing movement in all three axes.

### 3. Run Pick-and-Place Example (in another terminal)

```bash
rosrun gantry_robot pick_and_place_example.py
```

The robot will demonstrate a complete assembly task with:
- Approach movements
- Picking operations
- Placement operations
- Return to home

## Robot Specifications

### Axes and Limits

| Axis | Type | Range | Velocity | Effort |
|------|------|-------|----------|--------|
| X | Prismatic | -0.4 to 0.4 m | 1.0 m/s | 100 N |
| Y | Prismatic | -0.3 to 0.3 m | 1.0 m/s | 100 N |
| Z | Prismatic | -0.5 to 0.5 m | 1.0 m/s | 100 N |

### End-Effector

- **Type**: Suction cup gripper
- **Payload**: ~0.5 kg
- **Stroke**: Full workspace reach
- **Components**: 
  - Tool frame (tool0)
  - Gripper cup (gripper_cup)
  - Suction simulation (on/off in code)

## Usage Examples

### Basic Motion Control

```python
from gantry_robot.motion_example import GantryMotionController

controller = GantryMotionController()

# Move to position
controller.move_xyz(x=0.2, y=0.1, z=-0.3, duration=2.0)

# Demonstration
controller.demo_motion()
```

### Pick and Place Operations

```python
from gantry_robot.pick_and_place_example import PickAndPlaceController

controller = PickAndPlaceController()

# Simple pick and place
controller.pick_and_place(
    pick_x=0.2, pick_y=0.1,
    place_x=-0.2, place_y=0.2
)

# Complex assembly task
controller.demo_assembly_task()
```

## ROS Topics and Services

### Published Topics

- `/joint_states`: Current joint positions and velocities
- `/tf`: Transformation frames for all links

### Subscribed Topics

- `/joint_x_controller/command`: X-axis trajectory commands
- `/joint_y_controller/command`: Y-axis trajectory commands
- `/joint_z_controller/command`: Z-axis trajectory commands

### Action Interfaces

- `/joint_x_controller/follow_joint_trajectory`: X-axis trajectory action
- `/joint_y_controller/follow_joint_trajectory`: Y-axis trajectory action
- `/joint_z_controller/follow_joint_trajectory`: Z-axis trajectory action

## Motion Planning

### Joint Trajectory Control

The robot uses `JointTrajectoryController` from the `effort_controllers` package. Each axis has independent control with PID gains:

```yaml
gains:
  joint_x: {p: 1000.0, i: 500.0, d: 10.0}
  joint_y: {p: 1000.0, i: 500.0, d: 10.0}
  joint_z: {p: 1000.0, i: 500.0, d: 10.0}
```

Adjust these in `config/controllers.yaml` for different performance characteristics.

## Customization

### Modify Robot Dimensions

Edit `urdf/gantry_robot.urdf`:

```xml
<!-- Change joint limits -->
<limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>

<!-- Change link sizes -->
<box size="0.3 0.6 0.2"/>
```

### Add New Objects to Workspace

Edit `worlds/gantry_workspace.world` to add:
- Static obstacles
- Target positions
- Assembly surfaces

### Adjust Controller Parameters

Edit `config/controllers.yaml`:
- PID gains for responsiveness
- Velocity limits
- Trajectory constraints

## Troubleshooting

### Controller Not Responding

**Problem**: Controllers don't load or robot doesn't move

**Solution**:
```bash
# Check if controller manager is running
rosservice call /controller_manager/list_controllers

# Manually spawn controllers
rosservice call /controller_manager/load_controller joint_x_controller
rosservice call /controller_manager/switch_controller "{start_controllers: ['joint_x_controller']}"
```

### Gazebo Physics Issues

**Problem**: Robot falls through table or behaves erratically

**Solution**:
- Increase physics accuracy in `worlds/gantry_workspace.world`:
  ```xml
  <max_step_size>0.0001</max_step_size>
  ```
- Reduce real-time factor for slower simulation

### Missing Dependencies

**Problem**: "ModuleNotFoundError" or "Package not found"

**Solution**:
```bash
# Rebuild workspace
cd ~/catkin_ws
catkin_make --pkg gantry_robot
source devel/setup.bash

# Check dependencies
rosdep check gantry_robot
```

## Advanced Topics

### Custom Motion Profiles

Create complex trajectories in Python:

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

traj = JointTrajectory()
traj.joint_names = ['joint_x', 'joint_y', 'joint_z']

# Add waypoints
point1 = JointTrajectoryPoint()
point1.positions = [0.1, 0.1, 0.0]
point1.velocities = [0, 0, 0]
point1.time_from_start = rospy.Duration(1.0)

point2 = JointTrajectoryPoint()
point2.positions = [0.3, 0.2, -0.3]
point2.velocities = [0, 0, 0]
point2.time_from_start = rospy.Duration(3.0)

traj.points = [point1, point2]
```

### Implementing Collision Detection

Monitor `/gazebo/contact_states` to detect collisions with workspace boundaries or objects.

### Vision Integration

Add camera simulation to `gantry_robot.urdf`:

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="tool0"/>
  <child link="camera_link"/>
  <origin xyz="0 0 -0.15"/>
</joint>
```

Then add camera plugin in Gazebo launch file.

## Performance Tips

1. **Reduce Update Rate**: Lower publish frequency in robot_state_publisher if CPU usage is high
2. **Simulation Speed**: Increase real_time_factor in world file for faster testing
3. **Graphics**: Disable shadows in Gazebo for better performance
4. **Physics**: Use ODE solver (already configured)

## References

- [ROS Control Documentation](http://wiki.ros.org/ros_control)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [URDF Documentation](http://wiki.ros.org/urdf)
- [Joint Trajectory Controller](http://wiki.ros.org/joint_trajectory_controller)

## License

BSD 3-Clause License

## Contributing

Contributions welcome! Please:
1. Test thoroughly
2. Document changes
3. Follow ROS naming conventions
4. Submit pull requests

## Support

For issues or questions:
- Check troubleshooting section
- Review example code
- Check ROS logs: `rqt_console`
- Monitor robot state: `rqt_robot_steering`

---

**Happy Simulating!** 🤖

For more complex tasks, consider integrating MoveIt! for motion planning:
```bash
sudo apt-get install ros-noetic-moveit
```
