#!/bin/bash
# Setup script for ROS Gantry Robot

set -e  # Exit on error

echo "╔════════════════════════════════════════════════════════╗"
echo "║   ROS GANTRY ROBOT - SETUP SCRIPT                      ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""

# Check if ROS is installed
if ! command -v roscore &> /dev/null; then
    echo "❌ ERROR: ROS is not installed"
    echo "Please install ROS Noetic first:"
    echo "  http://wiki.ros.org/noetic/Installation/Ubuntu"
    exit 1
fi

ROS_DISTRO=$(grep -oP '(?<=ROS_DISTRO=)[^ ]*' /opt/ros/*/setup.bash | head -1)
echo "✓ ROS Distro: $ROS_DISTRO"

# Create catkin workspace if it doesn't exist
if [ ! -d "$HOME/catkin_ws" ]; then
    echo ""
    echo "Creating catkin workspace..."
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make
    echo "✓ Catkin workspace created"
else
    echo "✓ Catkin workspace already exists"
fi

# Copy package
echo ""
echo "Installing gantry_robot package..."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
if [ -d "$SCRIPT_DIR/gantry_robot" ]; then
    cp -r "$SCRIPT_DIR/gantry_robot" ~/catkin_ws/src/
    echo "✓ Package copied"
else
    echo "❌ ERROR: gantry_robot directory not found"
    exit 1
fi

# Build the package
echo ""
echo "Building package..."
cd ~/catkin_ws
source /opt/ros/$ROS_DISTRO/setup.bash
catkin_make --pkg gantry_robot -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo "✓ Build successful"
else
    echo "❌ Build failed"
    exit 1
fi

# Setup bashrc
echo ""
echo "Configuring shell..."
if ! grep -q "catkin_ws/devel/setup.bash" ~/.bashrc; then
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    echo "✓ Added workspace source to ~/.bashrc"
else
    echo "✓ Workspace already sourced in ~/.bashrc"
fi

# Make scripts executable
echo ""
echo "Making scripts executable..."
chmod +x ~/catkin_ws/src/gantry_robot/src/*.py
echo "✓ Scripts are executable"

# Install dependencies
echo ""
echo "Checking ROS dependencies..."
cd ~/catkin_ws/src/gantry_robot

MISSING_DEPS=0
for dep in gazebo_ros gazebo_plugins controller_manager joint_state_controller joint_trajectory_controller effort_controllers robot_state_publisher joint_state_publisher; do
    if ! dpkg -l | grep -q "ros-.*-$dep"; then
        echo "  ⚠ Missing: ros-$ROS_DISTRO-$dep"
        MISSING_DEPS=1
    fi
done

if [ $MISSING_DEPS -eq 1 ]; then
    echo ""
    echo "Installing missing dependencies..."
    sudo apt-get update
    sudo apt-get install -y \
        ros-$ROS_DISTRO-gazebo-ros \
        ros-$ROS_DISTRO-gazebo-plugins \
        ros-$ROS_DISTRO-controller-manager \
        ros-$ROS_DISTRO-joint-state-controller \
        ros-$ROS_DISTRO-joint-trajectory-controller \
        ros-$ROS_DISTRO-effort-controllers \
        ros-$ROS_DISTRO-robot-state-publisher \
        ros-$ROS_DISTRO-joint-state-publisher
    echo "✓ Dependencies installed"
else
    echo "✓ All dependencies are installed"
fi

# Verification
echo ""
echo "╔════════════════════════════════════════════════════════╗"
echo "║                   SETUP COMPLETE                        ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""
echo "Next steps:"
echo ""
echo "1. Reload shell configuration:"
echo "   source ~/.bashrc"
echo ""
echo "2. Launch the simulation:"
echo "   roslaunch gantry_robot gazebo.launch"
echo ""
echo "3. In another terminal, run a demo:"
echo "   rosrun gantry_robot motion_example.py"
echo ""
echo "For more information, see:"
echo "  - QUICKSTART.md  (5-minute quick start)"
echo "  - README.md      (comprehensive documentation)"
echo ""
