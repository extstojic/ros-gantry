# Quick Start Guide - ROS Gantry Robot

## 5-Minute Setup

### Terminal 1: Launch Simulation
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch gantry_robot gazebo.launch
```

Wait for Gazebo to open and robot to spawn (you'll see it in the 3D view).

### Terminal 2: Run a Demo
```bash
cd ~/catkin_ws
source devel/setup.bash

# Option A: Motion demo
rosrun gantry_robot motion_example.py

# Option B: Pick and place demo
rosrun gantry_robot pick_and_place_example.py

# Option C: Interactive control
rosrun gantry_robot interactive_control.py
```

## What You'll See

**Gazebo Window**: Shows the robot and workspace with objects
- Orange box: X-axis carriage
- Blue box: Y-axis carriage  
- Green box: Z-axis carriage
- Red cylinder: End-effector
- Yellow disc: Gripper cup

**Terminal Output**: Shows robot status and what it's doing

## Interactive Control Commands

```
move 0.2 0.1 -0.3   # Move to X=0.2, Y=0.1, Z=-0.3
x 0.3               # Move X axis only
home                # Return to origin
demo                # Show motion demo
pick                # Show pick and place
help                # Show all commands
```

## Troubleshooting

**Q: Gazebo crashes on startup**
```bash
# Try with verbose output
roslaunch gantry_robot gazebo.launch --screen
```

**Q: Robot doesn't move**
```bash
# Check if controllers are loaded
rosservice call /controller_manager/list_controllers
```

**Q: "Action server not available"**
- Wait 10 seconds after launching simulation
- Check that gazebo.launch completed successfully
- Verify all dependencies are installed: `catkin_make`

**Q: Can't run Python scripts**
```bash
# Make executable
chmod +x ~/catkin_ws/src/gantry_robot/src/*.py
```

## Next Steps

1. **Modify the motion**: Edit `motion_example.py` to create custom trajectories
2. **Add objects**: Edit `worlds/gantry_workspace.world` to add obstacles
3. **Tune controller**: Adjust PID gains in `config/controllers.yaml`
4. **Integrate MoveIt**: Add motion planning for complex tasks

## Full Documentation

See `README.md` in the package root for comprehensive documentation.
