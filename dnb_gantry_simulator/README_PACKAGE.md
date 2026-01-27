# Gantry Simulator Package Structure

This package implements a 3-axis Cartesian gantry robot simulator for drag&bot.

## Directory Structure

```
dnb_gantry_simulator/
├── CMakeLists.txt           # Build configuration with mesh installation
├── package.xml              # ROS package dependencies
├── README.md                # This file
├── launch/
│   ├── start.launch         # Main launch file
│   └── stop.launch          # Shutdown launch file
├── src/
│   ├── gantry_node.cpp      # ROS node entry point
│   ├── gantry_driver.cpp    # ROS service interface
│   └── gantry_controller.cpp# Motion simulation logic
├── include/
│   └── dnb_gantry_simulator/
│       ├── gantry_driver.h  # Driver interface
│       └── gantry_controller.h  # Controller interface
├── urdf/
│   └── gantry.urdf          # Robot description with box geometry
├── meshes/
│   └── gantry/
│       ├── visual/          # Visual mesh files (DAE format)
│       │   ├── base.dae
│       │   └── x_carriage.dae
│       └── collision/       # Collision mesh files
├── module_config/
│   ├── module.yaml          # Old module definition (kept for compatibility)
│   ├── module_generic.yaml  # Generic module definition
│   ├── config/              # Configuration variants
│   │   └── standard.yaml    # Standard gantry configuration
│   ├── robot/               # Robot model definitions
│   │   └── standard.yaml    # Standard gantry robot definition
│   └── error_codes/
│       └── error_code_table.csv
├── fnb/                     # Function blocks for drag&bot UI
├── srv/
│   └── MoveGantry.srv       # Service definition
└── test/
    └── identifier_list.json # Function block identifiers
```

## Key Features

- **3-Axis Cartesian Gantry**: X, Y, Z axes with configurable limits
- **ROS Integration**: Services, joint state publishing, TF2 support
- **drag&bot Compatible**: Module configuration for integration as main robot
- **Visualization**: URDF with geometry and mesh support
- **Configurable**: Workspace limits and speed via launch parameters

## Workspace Configuration

The gantry uses the following default workspace limits:
- X: -0.4m to 0.4m (800mm total travel)
- Y: -0.3m to 0.3m (600mm total travel)
- Z: -0.5m to 0.1m (600mm total travel)
- Max Speed: 0.2 m/s (configurable)

## Module Organization

This package follows the drag&bot driver pattern:

1. **module_generic.yaml**: Generic configuration template
2. **module_config/config/**: Variant-specific configurations
3. **module_config/robot/**: Robot model definitions
4. **meshes/**: Visual and collision geometry files

## Mesh Files

Mesh files are provided in DAE (Collada) format for compatibility:
- Located in `meshes/gantry/visual/` and `meshes/gantry/collision/`
- Include geometry for base, X-carriage, Y-carriage, Z-column, and tool
- Can be extended with actual CAD meshes or STL files

## Building and Installation

```bash
# In Docker or catkin workspace
catkin build dnb_gantry_simulator

# Meshes are automatically installed to:
# $INSTALL_PREFIX/share/dnb_gantry_simulator/meshes/
```

## Testing with ROS

```bash
# Launch the simulator
roslaunch dnb_gantry_simulator start.launch

# Call move service
rosservice call /dnb_gantry_simulator/move "{x: 0.1, y: 0.0, z: 0.0, speed: 0.05, speed_unit: 'M/S'}"

# Monitor joint states
rostopic echo /joint_states

# View robot in RViz
rosrun rviz rviz -d config.rviz
```

## Integration with drag&bot

1. Package is installed to `/root/dnb_bundle/install/` in Docker
2. Module configuration loaded from `module_config/module_generic.yaml`
3. Robot visible in operator panel when configured
4. Communicates via ROS services to `/dnb_gantry_simulator/move`

## Troubleshooting

- **Robot not visible**: Check that meshes are installed and URDF paths are correct
- **Services not responding**: Verify `dnb_gantry_simulator` node is running
- **Joint states not publishing**: Check `joint_state_publisher` is running
- **Docker build fails**: Ensure all dependencies are listed in `package.xml`
