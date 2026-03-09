# ROS Gantry Robot Simulator

3-axis Cartesian robot simulation for drag&bot testing and assembly tasks.

## Requirements

- ROS Noetic
- drag&bot packages (dnb_msgs, robot_movement_interface, dnb_remote_robot_control)

## Installation

```bash
cd ~/dnb_catkin_ws/src
git clone <this-repo>
cd ~/dnb_catkin_ws
catkin_make
source devel/setup.bash
```

## Launch

```bash
roslaunch dnb_gantry_simulator start.launch
```

This starts the simulator and integrates with drag&bot UI.

## Robot Specs

- X axis: -0.4 to 0.4 m
- Y axis: -0.3 to 0.3 m  
- Z axis: -0.4 to 0.4 m
- Max speed: 0.2 m/s
- End effector: Parallel jaw gripper

## Control

The robot accepts standard drag&bot commands via `/command_list` topic.

Use the drag&bot UI for jogging and motion control.

## Examples

Run basic motion demo:
```bash
rosrun dnb_gantry_simulator move_example.py
```

Run pick and place demo:
```bash
rosrun dnb_gantry_simulator pick_and_place_example.py
```

Send manual command:
```bash
rostopic pub /command_list robot_movement_interface/CommandList "{
  replace_previous_commands: true,
  commands: [{command_id: 1, command_type: 'LIN', 
    pose: [0.1, 0.05, -0.2, 0, 0, 0], velocity: [0.1], velocity_type: 'M/S'}]}"
```

## Configuration

Adjust workspace limits:
```bash
roslaunch dnb_gantry_simulator start.launch min_x:=-0.5 max_x:=0.5
```
