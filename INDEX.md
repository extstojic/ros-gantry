# ROS Gantry Robot - Complete Index

## 📚 Documentation Files (Read These First!)

### Quick Reference
- **[QUICKSTART.md](QUICKSTART.md)** ⚡ - **START HERE!** 5-minute setup and basic usage
- **[README.md](README.md)** 📖 - Comprehensive documentation with all details
- **[PROJECT.md](PROJECT.md)** 🎯 - Project overview and feature summary
- **[ARCHITECTURE.md](ARCHITECTURE.md)** 🏗️ - System architecture and diagrams

## 🚀 Getting Started

### Step 1: Automated Setup (Recommended)
```bash
cd /root/Desktop/ros-gantry
bash setup.sh
source ~/.bashrc
```

### Step 2: Launch Simulation
```bash
roslaunch gantry_robot gazebo.launch
```

### Step 3: Run a Demo (in new terminal)
```bash
# Option A: Motion demonstration
rosrun gantry_robot motion_example.py

# Option B: Pick and place demo
rosrun gantry_robot pick_and_place_example.py

# Option C: Interactive control
rosrun gantry_robot interactive_control.py
```

## 📦 Package Structure

### Robot Package: `gantry_robot/`

#### Core Files
- `package.xml` - Package metadata and dependencies
- `CMakeLists.txt` - Build configuration

#### URDF Model: `urdf/`
- `gantry_robot.urdf` - Complete robot model with 3 axes and end-effector

#### Simulation: `worlds/`
- `gantry_workspace.world` - Gazebo world with physics and objects

#### Configuration: `config/`
- `controllers.yaml` - PID controller parameters for X, Y, Z axes

#### Launch Files: `launch/`
- `gazebo.launch` - Single command to start full simulation

#### Python Scripts: `src/`
- `motion_example.py` - Basic motion control demonstration
- `pick_and_place_example.py` - Pick-and-place assembly tasks
- `interactive_control.py` - Command-line interactive interface

## 🤖 Robot Specifications

| Property | Value |
|----------|-------|
| **Type** | 3-Axis Cartesian |
| **X Range** | -0.4 to 0.4 m |
| **Y Range** | -0.3 to 0.3 m |
| **Z Range** | -0.5 to 0.5 m |
| **Velocity** | 1.0 m/s per axis |
| **Effort** | 100 N per axis |
| **Gripper** | Suction cup |
| **Physics** | ODE (Gazebo) |
| **Control** | JointTrajectoryController |
| **ROS Version** | Noetic |

## 💻 Python API Examples

### Basic Motion
```python
from gantry_robot.motion_example import GantryMotionController

controller = GantryMotionController()
controller.move_xyz(x=0.2, y=0.1, z=-0.3, duration=2.0)
```

### Pick and Place
```python
from gantry_robot.pick_and_place_example import PickAndPlaceController

controller = PickAndPlaceController()
controller.pick_and_place(pick_x=0.2, pick_y=0.1, 
                         place_x=-0.2, place_y=0.2)
```

### Interactive Control
```bash
rosrun gantry_robot interactive_control.py
gantry> move 0.2 0.1 -0.3    # Move to position
gantry> home                 # Return to origin
gantry> x 0.3                # Move X axis only
gantry> demo                 # Run motion demo
```

## 🎮 Interactive Commands Reference

```
MOTION COMMANDS:
  move X Y Z     - Move to absolute position
  x POS          - Move X axis to position
  y POS          - Move Y axis to position
  z POS          - Move Z axis to position
  home           - Return to origin (0,0,0)

DEMONSTRATION:
  demo           - Show motion capabilities
  pick           - Show pick and place operation
  limits         - Show joint limits

HELP:
  help           - Show all commands
  quit/exit      - Exit program

EXAMPLES:
  gantry> move 0.2 0.1 -0.3   # Move to (0.2, 0.1, -0.3)
  gantry> x 0.3                # Set X=0.3 (Y,Z unchanged)
  gantry> home                 # Return to (0,0,0)
  gantry> demo                 # Show demo sequence
```

## 🔧 Customization Guide

### Modify Robot Dimensions
Edit `urdf/gantry_robot.urdf`:
- Change `<size>` tags for link dimensions
- Modify `<limit>` tags for joint ranges
- Adjust mass and inertia properties

### Adjust Controller Performance
Edit `config/controllers.yaml`:
- Change PID gains (p, i, d) for responsiveness
- Modify velocity and effort limits
- Tune trajectory constraints

### Add Workspace Objects
Edit `worlds/gantry_workspace.world`:
- Insert additional models
- Add obstacles or fixtures
- Modify lighting and physics

### Create Custom Motion Sequences
Create new Python files in `src/`:
```python
#!/usr/bin/env python3
import rospy
from motion_example import GantryMotionController

controller = GantryMotionController()
# Your custom motion sequence here
```

## 🐛 Troubleshooting

### "Action servers not available"
→ Wait 10 seconds after launching simulation
→ Verify gazebo.launch completed successfully

### "Robot doesn't move"
→ Check controllers: `rosservice call /controller_manager/list_controllers`
→ Check ROS logs: `rqt_console`

### "Gazebo crashes"
→ Run with verbose output: `roslaunch gantry_robot gazebo.launch --screen`
→ Check system resources (RAM, CPU)

### "Can't run Python scripts"
→ Make executable: `chmod +x ~/catkin_ws/src/gantry_robot/src/*.py`

See **[README.md](README.md)** for complete troubleshooting guide.

## 📊 ROS Topics

### Published Topics
- `/joint_states` - Current joint positions and velocities
- `/tf` - Transformation frames for all robot links

### Action Servers
- `/joint_x_controller/follow_joint_trajectory`
- `/joint_y_controller/follow_joint_trajectory`
- `/joint_z_controller/follow_joint_trajectory`

### Services
- `/controller_manager/list_controllers`
- `/controller_manager/load_controller`
- `/controller_manager/switch_controller`

## 🎯 Common Tasks

### Run Basic Motion Demo
```bash
roslaunch gantry_robot gazebo.launch
# In new terminal:
rosrun gantry_robot motion_example.py
```

### Run Pick and Place Demo
```bash
roslaunch gantry_robot gazebo.launch
# In new terminal:
rosrun gantry_robot pick_and_place_example.py
```

### Interactive Control
```bash
roslaunch gantry_robot gazebo.launch
# In new terminal:
rosrun gantry_robot interactive_control.py
```

### Monitor Robot State
```bash
rostopic echo /joint_states
```

### View 3D Robot Model
```bash
rviz -d <config_file>
# Or use default visualization in Gazebo
```

## 🚦 System Requirements

- **OS**: Ubuntu 20.04 (or compatible Linux)
- **ROS**: Noetic
- **Python**: 3.6+
- **Gazebo**: 11.x
- **RAM**: 4GB minimum (8GB recommended)
- **CPU**: Dual-core minimum (quad-core recommended)

## 📖 Additional Resources

### Online Documentation
- [ROS Control Framework](http://wiki.ros.org/ros_control)
- [Gazebo Simulation](http://gazebosim.org)
- [URDF Format](http://wiki.ros.org/urdf)
- [Joint Trajectory Controller](http://wiki.ros.org/joint_trajectory_controller)

### Related ROS Packages
- `moveit` - Advanced motion planning
- `rqt_robot_steering` - Manual robot control GUI
- `rqt_console` - ROS message viewer
- `rviz` - 3D visualization tool

## 🎓 Learning Path

1. **Start Here**: Read [QUICKSTART.md](QUICKSTART.md)
2. **Understand the System**: Read [ARCHITECTURE.md](ARCHITECTURE.md)
3. **Run Examples**: Execute motion_example.py and pick_and_place_example.py
4. **Interactive Control**: Use interactive_control.py to learn motion control
5. **Modify Code**: Create custom trajectories in Python
6. **Extend System**: Add new features (vision, MoveIt!, etc.)

## 📝 File Reference

```
/root/Desktop/ros-gantry/
├── README.md                           # Full documentation
├── QUICKSTART.md                       # 5-minute setup
├── PROJECT.md                          # Project overview
├── ARCHITECTURE.md                     # System architecture
├── setup.sh                            # Automated setup
│
└── gantry_robot/                       # ROS Package
    ├── package.xml                    # Package metadata
    ├── CMakeLists.txt                 # Build config
    │
    ├── urdf/
    │   └── gantry_robot.urdf          # Robot model
    │
    ├── launch/
    │   └── gazebo.launch              # Gazebo launcher
    │
    ├── config/
    │   └── controllers.yaml           # Controller config
    │
    ├── worlds/
    │   └── gantry_workspace.world     # Gazebo world
    │
    └── src/
        ├── motion_example.py          # Motion demo
        ├── pick_and_place_example.py  # Pick and place
        └── interactive_control.py     # Interactive interface
```

## ✅ What's Included

✓ Complete ROS package with proper structure
✓ URDF robot model with 3 axes and gripper
✓ Gazebo simulation environment with physics
✓ PID-tuned trajectory controllers
✓ Three working example applications
✓ Interactive command-line interface
✓ Comprehensive documentation
✓ Automated setup script
✓ System architecture diagrams

## 🎉 Quick Commands Cheat Sheet

```bash
# Setup (one time)
bash /root/Desktop/ros-gantry/setup.sh
source ~/.bashrc

# Launch simulation
roslaunch gantry_robot gazebo.launch

# Run examples (in new terminal)
rosrun gantry_robot motion_example.py          # Motion demo
rosrun gantry_robot pick_and_place_example.py  # Pick and place
rosrun gantry_robot interactive_control.py     # Interactive

# Check system status
rosservice call /controller_manager/list_controllers
rostopic echo /joint_states
```

---

**Ready to begin?** Start with [QUICKSTART.md](QUICKSTART.md) for fast setup! 🚀

