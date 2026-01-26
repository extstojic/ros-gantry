# ROS Gantry Robot - System Architecture

## Component Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS GANTRY ROBOT SYSTEM                  │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                      GAZEBO SIMULATOR                        │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              GANTRY ROBOT (URDF)                     │   │
│  ├──────────────────────────────────────────────────────┤   │
│  │                                                      │   │
│  │  ┌─────────────────────────────────────────────┐   │   │
│  │  │         BASE_LINK (Fixed Frame)            │   │   │
│  │  │  Size: 1.0m x 1.0m x 1.0m (gray box)      │   │   │
│  │  └────────────┬────────────────────────────────┘   │   │
│  │               │ joint_x (Prismatic)                 │   │
│  │               │ Range: ±0.4m, 100N effort          │   │
│  │               ▼                                      │   │
│  │  ┌──────────────────────────────────────────┐      │   │
│  │  │      X_CARRIAGE (Orange box)             │      │   │
│  │  │  Size: 0.3m x 0.6m x 0.2m               │      │   │
│  │  └────────────┬─────────────────────────────┘      │   │
│  │               │ joint_y (Prismatic)                 │   │
│  │               │ Range: ±0.3m, 100N effort          │   │
│  │               ▼                                      │   │
│  │  ┌──────────────────────────────────────────┐      │   │
│  │  │      Y_CARRIAGE (Blue box)               │      │   │
│  │  │  Size: 0.6m x 0.3m x 0.2m               │      │   │
│  │  └────────────┬─────────────────────────────┘      │   │
│  │               │ joint_z (Prismatic)                 │   │
│  │               │ Range: ±0.5m, 100N effort          │   │
│  │               ▼                                      │   │
│  │  ┌──────────────────────────────────────────┐      │   │
│  │  │      Z_CARRIAGE (Green box)              │      │   │
│  │  │  Size: 0.2m x 0.2m x 0.3m               │      │   │
│  │  └────────────┬─────────────────────────────┘      │   │
│  │               │ tool_joint (Fixed)                  │   │
│  │               ▼                                      │   │
│  │  ┌──────────────────────────────────────────┐      │   │
│  │  │      TOOL0 (Red cylinder)                │      │   │
│  │  │  End-effector frame                      │      │   │
│  │  └────────────┬─────────────────────────────┘      │   │
│  │               │ gripper_joint (Fixed)               │   │
│  │               ▼                                      │   │
│  │  ┌──────────────────────────────────────────┐      │   │
│  │  │    GRIPPER_CUP (Yellow disc)             │      │   │
│  │  │  Suction cup for pick and place          │      │   │
│  │  └──────────────────────────────────────────┘      │   │
│  │                                                      │   │
│  └──────────────────────────────────────────────────────┘   │
│                                                               │
│  ┌──────────────────────────────────────────────────────┐   │
│  │         WORKSPACE OBJECTS (Sample Objects)          │   │
│  ├──────────────────────────────────────────────────────┤   │
│  │  • Object_1 (Blue cube) - Position: (0.2, 0.1, 0.1) │   │
│  │  • Object_2 (Green cube) - Position: (-0.2, 0.1, 0.1)  │   │
│  │  • Work Table (Static) - Under robot               │   │
│  └──────────────────────────────────────────────────────┘   │
│                                                               │
└─────────────────────────────────────────────────────────────┘
         ▲                        ▲                ▲
         │                        │                │
         │                        │                │
    PhysX Topics            Joint States       TF Frames
```

## Data Flow Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                        USER APPLICATION                           │
│  (motion_example.py, pick_and_place_example.py, etc.)            │
└───────────┬──────────────────────────────────────────────────────┘
            │ ROS Action Messages (FollowJointTrajectory)
            │
┌───────────▼──────────────────────────────────────────────────────┐
│                   JOINT TRAJECTORY CONTROLLERS                    │
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐   │
│  │ joint_x_ctrl    │ │ joint_y_ctrl    │ │ joint_z_ctrl    │   │
│  │ effort_ctrlr    │ │ effort_ctrlr    │ │ effort_ctrlr    │   │
│  │ PID: 1000/500/10│ │ PID: 1000/500/10│ │ PID: 1000/500/10│   │
│  └────────┬────────┘ └────────┬────────┘ └────────┬────────┘   │
└───────────┼────────────────────┼────────────────────┼────────────┘
            │ Effort Commands    │                   │
            │ (Joint Forces)     │                   │
            │                    │                   │
┌───────────▼────────────────────▼────────────────────▼────────────┐
│                     GAZEBO PHYSICS ENGINE (ODE)                   │
│  Integrates forces → calculates accelerations → updates positions │
└───────────┬────────────────────┬────────────────────┬────────────┘
            │                    │                    │
      /joint_states          /tf (TF tree)        /gazebo/* topics
      (joint pos/vel)      (transformation)       (physics data)
            │                    │                    │
┌───────────▼────────────────────▼────────────────────▼────────────┐
│                    ROS MESSAGE TOPICS                             │
│  • /joint_states - Current position and velocity of all joints   │
│  • /tf - Transformation frames for robot visualization           │
│  • /gazebo/contact_states - Collision information                │
│  • /clock - Simulation time                                      │
└──────────────────────────────────────────────────────────────────┘
            │
┌───────────▼──────────────────────────────────────────────────────┐
│                  VISUALIZATION & MONITORING                       │
│  • RViz - 3D visualization of robot and TF frames                 │
│  • Gazebo GUI - Physics simulation view                           │
│  • rqt_robot_steering - Manual joint control                     │
└──────────────────────────────────────────────────────────────────┘
```

## Control Flow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│ USER SENDS COMMAND                                          │
│ Ex: move_xyz(x=0.2, y=0.1, z=-0.3, duration=2.0)          │
└──────────────────┬──────────────────────────────────────────┘
                   │
         ┌─────────▼──────────┐
         │ Create Trajectory  │
         │ with 3 waypoints:  │
         │ - joint_x goal     │
         │ - joint_y goal     │
         │ - joint_z goal     │
         └─────────┬──────────┘
                   │
         ┌─────────▼────────────────────────────┐
         │ Send to Action Servers (Parallel)    │
         │ /joint_x_controller/follow_...       │
         │ /joint_y_controller/follow_...       │
         │ /joint_z_controller/follow_...       │
         └─────────┬────────────────────────────┘
                   │
         ┌─────────▼────────────────────────────┐
         │ Controllers Calculate Effort         │
         │ (PID loop at 50 Hz)                  │
         │ Error = Goal - Current Position      │
         │ Effort = K_p*E + K_i*∫E + K_d*dE/dt │
         └─────────┬────────────────────────────┘
                   │
         ┌─────────▼────────────────────────────┐
         │ Gazebo Integrates Physics            │
         │ F = m*a (Newton's 2nd law)           │
         │ x(t+dt) = x(t) + v*dt + 0.5*a*dt²   │
         └─────────┬────────────────────────────┘
                   │
         ┌─────────▼────────────────────────────┐
         │ Publish Joint States                 │
         │ /joint_states (50 Hz)                │
         │ Broadcast TF                         │
         └─────────┬────────────────────────────┘
                   │
         ┌─────────▼────────────────────────────┐
         │ Loop Until Goal Reached              │
         │ tolerance: ±0.02 m                   │
         │ wait_for_result()                    │
         └─────────┬────────────────────────────┘
                   │
         ┌─────────▼──────────┐
         │ GOAL REACHED!      │
         │ Return to user     │
         └────────────────────┘
```

## Joint Hierarchy Tree

```
world
  └── base_link (XYZ: 0, 0, 0.5m)
      │
      ├── [joint_x: Prismatic] →
      │   └── x_carriage (XYZ: varies, 0, 0)
      │       │
      │       ├── [joint_y: Prismatic] →
      │       │   └── y_carriage (XYZ: 0, varies, 0)
      │       │       │
      │       │       ├── [joint_z: Prismatic] →
      │       │       │   └── z_carriage (XYZ: 0, 0, varies)
      │       │       │       │
      │       │       │       ├── [tool_joint: Fixed] →
      │       │       │       │   └── tool0 (XYZ: 0, 0, -0.15)
      │       │       │       │       │
      │       │       │       │       └── [gripper_joint: Fixed] →
      │       │       │       │           └── gripper_cup (XYZ: 0, 0, -0.1)
```

## Motion Constraints

```
X-AXIS (joint_x)                Y-AXIS (joint_y)              Z-AXIS (joint_z)
├─ Range: -0.4 to +0.4m        ├─ Range: -0.3 to +0.3m       ├─ Range: -0.5 to +0.5m
├─ Max Velocity: 1.0 m/s       ├─ Max Velocity: 1.0 m/s      ├─ Max Velocity: 1.0 m/s
├─ Max Effort: 100 N           ├─ Max Effort: 100 N          ├─ Max Effort: 100 N
├─ Damping: 10 Ns/m            ├─ Damping: 10 Ns/m           ├─ Damping: 10 Ns/m
└─ Friction: 5 N               └─ Friction: 5 N              └─ Friction: 5 N

Workspace Envelope:
┌────────────────────────────────────┐
│ X: ±0.4m × Y: ±0.3m × Z: ±0.5m   │
│ Total Volume: 0.8 × 0.6 × 1.0 m³  │
│ Reachable Points: 0.48 m³          │
└────────────────────────────────────┘
```

## Package Dependencies

```
gantry_robot
├── ROS Core
│   ├── rospy (Python support)
│   ├── roscpp (C++ support)
│   └── std_msgs (standard messages)
│
├── Simulation
│   ├── gazebo_ros (ROS-Gazebo integration)
│   ├── gazebo_plugins (Gazebo physics plugins)
│   └── gazebo (physics engine: ODE)
│
├── Control
│   ├── controller_manager (manages controllers)
│   ├── effort_controllers (force-based control)
│   ├── joint_trajectory_controller (trajectory following)
│   ├── joint_state_controller (state publishing)
│   └── ros_control (control framework)
│
├── Messages
│   ├── geometry_msgs (geometric primitives)
│   ├── trajectory_msgs (trajectory definitions)
│   ├── sensor_msgs (sensor data)
│   └── control_msgs (control commands)
│
├── Visualization
│   ├── rviz (3D visualization)
│   ├── robot_state_publisher (TF broadcasting)
│   ├── joint_state_publisher (joint state aggregation)
│   └── tf/tf2 (transformation library)
│
└── Description
    └── urdf/xacro (robot description formats)
```

---

This diagram shows the complete architecture and data flow of the ROS Gantry Robot System!
