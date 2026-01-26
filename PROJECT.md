# ROS Gantry Robot Simulation - Project Overview

## Project Summary

A complete ROS simulation package for a 3-axis Cartesian (Gantry) robot designed for industrial pick-and-place operations and assembly tasks. The package includes robot modeling, Gazebo physics simulation, motion control, and example applications.

## What's Included

### 📦 Core Components

1. **Robot Model** (`urdf/gantry_robot.urdf`)
   - 3 prismatic joints (X, Y, Z axes)
   - Realistic mass and inertia properties
   - End-effector with gripper cup
   - Color-coded visualization

2. **Simulation Environment** (`worlds/gantry_workspace.world`)
   - Gazebo physics engine (ODE)
   - Work surface with sample objects
   - Lighting and camera views
   - 2 pick-and-place target objects

3. **Control System** (`config/controllers.yaml`)
   - JointTrajectoryController for each axis
   - PID-tuned motion controllers
   - Velocity and effort limits

4. **Launch Files** (`launch/gazebo.launch`)
   - Single-command Gazebo startup
   - Automatic controller loading
   - Robot spawning and initialization

### 🎯 Example Applications

1. **motion_example.py** - Basic motion demonstration
   - Shows all 3-axis movement capabilities
   - Demonstrates rectangular path
   - Motion control patterns

2. **pick_and_place_example.py** - Assembly task simulation
   - Complete pick-and-place cycle
   - Gripper simulation (open/close)
   - Multi-task assembly demonstration

3. **interactive_control.py** - Manual robot control
   - Command-line interface
   - Real-time position control
   - Built-in demo sequences

### 📚 Documentation

1. **README.md** - Comprehensive documentation
   - Installation and setup
   - Feature descriptions
   - Usage examples
   - Troubleshooting guide

2. **QUICKSTART.md** - 5-minute quick start
   - Fast setup instructions
   - Essential commands
   - Common issues

3. **setup.sh** - Automated setup script
   - Dependency installation
   - Workspace configuration
   - Package building

## Directory Structure

```
ros-gantry/
├── setup.sh                           # Automated setup script
├── README.md                          # Full documentation
├── QUICKSTART.md                      # Quick start guide
│
└── gantry_robot/                      # ROS package
    ├── CMakeLists.txt                # Build configuration
    ├── package.xml                   # Package metadata
    │
    ├── urdf/
    │   └── gantry_robot.urdf         # Robot model definition
    │
    ├── launch/
    │   └── gazebo.launch             # Gazebo launcher
    │
    ├── config/
    │   └── controllers.yaml          # Controller configuration
    │
    ├── worlds/
    │   └── gantry_workspace.world    # Gazebo world file
    │
    └── src/
        ├── motion_example.py          # Basic motion demo
        ├── pick_and_place_example.py  # Pick and place demo
        └── interactive_control.py     # Interactive interface
```

## Key Features

✅ **3-Axis Linear Motion**
- Independent X, Y, Z movement
- Smooth trajectory control
- Configurable velocity and acceleration

✅ **Pick-and-Place Operations**
- Gripper simulation (on/off)
- Multi-object handling
- Approach/retreat motions

✅ **ROS-Standard Interface**
- JointTrajectoryController
- Standard ROS messages
- Action-based control

✅ **Physics Simulation**
- ODE physics engine
- Collision detection
- Realistic dynamics

✅ **Easy to Extend**
- Modular architecture
- Clean code structure
- Well-commented examples

## Getting Started

### Quick Setup (30 seconds)
```bash
cd /root/Desktop/ros-gantry
bash setup.sh
source ~/.bashrc
```

### Launch Simulation (Terminal 1)
```bash
roslaunch gantry_robot gazebo.launch
```

### Run Example (Terminal 2)
```bash
rosrun gantry_robot motion_example.py
```

## Usage Examples

### Python API
```python
from gantry_robot.motion_example import GantryMotionController

controller = GantryMotionController()
controller.move_xyz(x=0.2, y=0.1, z=-0.3, duration=2.0)
```

### Command Line (Interactive)
```bash
rosrun gantry_robot interactive_control.py

gantry> move 0.2 0.1 -0.3
gantry> home
gantry> pick
gantry> demo
```

### Direct ROS Commands
```bash
# Publish trajectory
rostopic pub /joint_x_controller/command trajectory_msgs/JointTrajectory ...

# Check controller status
rosservice call /controller_manager/list_controllers

# View robot state
rostopic echo /joint_states
```

## Specifications

| Property | Value |
|----------|-------|
| **Type** | 3-axis Cartesian (Gantry) |
| **X Range** | -0.4 to 0.4 m |
| **Y Range** | -0.3 to 0.3 m |
| **Z Range** | -0.5 to 0.5 m |
| **Max Velocity** | 1.0 m/s per axis |
| **Max Effort** | 100 N per axis |
| **Gripper** | Suction cup (simulation) |
| **End-Effector Payload** | ~0.5 kg |
| **Physics Engine** | ODE |
| **ROS Version** | Noetic |
| **Control Type** | JointTrajectoryController |

## Customization Options

### Modify Joint Limits
Edit `urdf/gantry_robot.urdf`:
```xml
<limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
```

### Adjust Controller Gains
Edit `config/controllers.yaml`:
```yaml
gains:
  joint_x: {p: 1000.0, i: 500.0, d: 10.0}
```

### Add Workspace Objects
Edit `worlds/gantry_workspace.world` to insert models or obstacles

### Tune Physics
Modify `worlds/gantry_workspace.world`:
```xml
<max_step_size>0.001</max_step_size>
<real_time_factor>1</real_time_factor>
```

## Performance Characteristics

- **Update Rate**: 50 Hz (joint state publisher)
- **Control Rate**: 50 Hz (controller manager)
- **Physics Timestep**: 1 ms
- **Motion Accuracy**: ±0.02 m (configurable)
- **Typical CPU Load**: 20-30% (single core)

## Advanced Features

### Motion Planning
Integrate MoveIt! for trajectory planning:
```bash
sudo apt-get install ros-noetic-moveit
```

### Vision Integration
Add camera to gripper in URDF and configure Gazebo camera plugin

### Force/Torque Feedback
Monitor joint efforts from `/joint_states` topic

### Collision Detection
Subscribe to `/gazebo/contact_states` for contact events

## Troubleshooting

**Robot not moving?**
- Check controller status: `rosservice call /controller_manager/list_controllers`
- Verify action servers are available
- Check ROS logs: `rqt_console`

**Physics issues?**
- Increase timestep accuracy in world file
- Check gravity settings
- Verify contact parameters

**Missing dependencies?**
- Run: `rosdep check gantry_robot`
- Install: `rosdep install gantry_robot`

## Next Steps

1. **Explore the Code**: Read the example scripts
2. **Create Custom Trajectories**: Modify motion_example.py
3. **Add Obstacles**: Edit the world file
4. **Integrate MoveIt!**: Add motion planning
5. **Add Vision**: Camera-based object detection

## References

- [ROS Control Docs](http://wiki.ros.org/ros_control)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [URDF Format](http://wiki.ros.org/urdf)
- [Joint Trajectory Controller](http://wiki.ros.org/joint_trajectory_controller)

## License

BSD 3-Clause License

---

**Ready to start?** See QUICKSTART.md for 5-minute setup instructions!
