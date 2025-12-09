#!/bin/bash
# Safe Gazebo startup script for four_wheel_bot
# This script handles common Gazebo issues and provides fallback options

set -e  # Exit on any error

echo "=== Four Wheel Bot - Safe Gazebo Startup ==="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if we're in the right directory
if [ ! -f "package.xml" ]; then
    print_error "Please run this script from the four_wheel_bot package directory"
    exit 1
fi

# Check ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    print_error "ROS2 environment not sourced. Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

print_status "ROS2 distribution: $ROS_DISTRO"

# Kill any existing Gazebo processes
print_status "Cleaning up existing Gazebo processes..."
pkill -f gazebo || true
pkill -f gzserver || true
pkill -f gzclient || true
sleep 2

# Check Gazebo installation
if ! command -v gazebo &> /dev/null; then
    print_error "Gazebo not found in PATH. Please install Gazebo Classic."
    exit 1
fi

print_status "Gazebo found: $(gazebo --version)"

# Set Gazebo environment variables
export GAZEBO_MODEL_PATH="$PWD/models:$GAZEBO_MODEL_PATH"
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH"
export GAZEBO_RESOURCE_PATH="/opt/ros/humble/share:$GAZEBO_RESOURCE_PATH"

print_status "Environment variables set"

# Build the package if needed
if [ ! -d "../build" ] || [ ! -d "../install" ]; then
    print_status "Building package..."
    cd ../..
    colcon build --packages-select four_wheel_bot
    source install/setup.bash
    cd src/four_wheel_bot
fi

# Source the workspace
if [ -f "../../install/setup.bash" ]; then
    source ../../install/setup.bash
    print_status "Workspace sourced"
else
    print_warning "Workspace not built. Please run: colcon build"
fi

# Choose launch method
LAUNCH_METHOD="robust"

case "$1" in
    "simple")
        LAUNCH_METHOD="simple"
        print_status "Using simple launch method"
        ;;
    "robust")
        LAUNCH_METHOD="robust"
        print_status "Using robust launch method (recommended)"
        ;;
    "debug")
        LAUNCH_METHOD="debug"
        print_status "Using debug launch method"
        ;;
    *)
        print_status "Usage: $0 [simple|robust|debug]"
        print_status "Defaulting to robust method"
        ;;
esac

# Launch based on method
case "$LAUNCH_METHOD" in
    "simple")
        print_status "Launching with simple method..."
        ros2 launch four_wheel_bot gazebo_model.launch.py
        ;;
    "robust")
        print_status "Launching with robust method..."
        ros2 launch four_wheel_bot gazebo_robust.launch.py
        ;;
    "debug")
        print_status "Launching with debug method..."
        ros2 launch four_wheel_bot gazebo_robust.launch.py verbose:=true gui:=true
        ;;
esac

print_status "Gazebo startup complete!"
