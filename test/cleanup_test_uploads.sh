#!/bin/bash
# Cleanup script to remove test upload files

# Base path on host
BASE_PATH="/home/yifan/wos"

echo "Cleaning up test upload files..."

# Clean up Gazebo models uploaded
if [ -d "$BASE_PATH/Assets/gazebo/models/uploaded" ]; then
    echo "Cleaning Gazebo models..."
    find "$BASE_PATH/Assets/gazebo/models/uploaded" -name "ur10_rg2_*" -type d -exec rm -rf {} + 2>/dev/null
    find "$BASE_PATH/Assets/gazebo/models/uploaded" -name "diff_drive_*" -type d -exec rm -rf {} + 2>/dev/null
    find "$BASE_PATH/Assets/gazebo/models/uploaded" -name "test_*" -type d -exec rm -rf {} + 2>/dev/null
    echo "  ✓ Gazebo models cleaned"
fi

# Clean up Gazebo launch files uploaded
if [ -d "$BASE_PATH/Assets/gazebo/launch/uploaded" ]; then
    echo "Cleaning Gazebo launch files..."
    find "$BASE_PATH/Assets/gazebo/launch/uploaded" -name "diff_drive.launch_*.py" -type f -exec rm -f {} + 2>/dev/null
    find "$BASE_PATH/Assets/gazebo/launch/uploaded" -name "test_*.py" -type f -exec rm -f {} + 2>/dev/null
    echo "  ✓ Gazebo launch files cleaned"
fi

# Clean up Gazebo worlds uploaded
if [ -d "$BASE_PATH/Assets/gazebo/worlds/uploaded" ]; then
    echo "Cleaning Gazebo worlds..."
    find "$BASE_PATH/Assets/gazebo/worlds/uploaded" -name "diff_drive_*.sdf" -type f -exec rm -f {} + 2>/dev/null
    find "$BASE_PATH/Assets/gazebo/worlds/uploaded" -name "test_*.sdf" -type f -exec rm -f {} + 2>/dev/null
    echo "  ✓ Gazebo worlds cleaned"
fi

# Clean up URDF uploaded files
if [ -d "$BASE_PATH/Assets/urdf/uploaded" ]; then
    echo "Cleaning URDF files..."
    find "$BASE_PATH/Assets/urdf/uploaded" -name "ur10_rg2_*" -type d -exec rm -rf {} + 2>/dev/null
    find "$BASE_PATH/Assets/urdf/uploaded" -name "rrbot_*" -type d -exec rm -rf {} + 2>/dev/null
    find "$BASE_PATH/Assets/urdf/uploaded" -name "test_*" -type d -exec rm -rf {} + 2>/dev/null
    echo "  ✓ URDF files cleaned"
fi

# Clean up CoppeliaSim models uploaded
if [ -d "$BASE_PATH/Assets/coppeliasim/models/uploaded" ]; then
    echo "Cleaning CoppeliaSim models..."
    find "$BASE_PATH/Assets/coppeliasim/models/uploaded" -name "test_*" -exec rm -f {} + 2>/dev/null
    echo "  ✓ CoppeliaSim models cleaned"
fi

# Clean up CoppeliaSim scenes uploaded
if [ -d "$BASE_PATH/Assets/coppeliasim/scenes/uploaded" ]; then
    echo "Cleaning CoppeliaSim scenes..."
    find "$BASE_PATH/Assets/coppeliasim/scenes/uploaded" -name "IoTRemoteLab_*.ttt" -type f -exec rm -f {} + 2>/dev/null
    find "$BASE_PATH/Assets/coppeliasim/scenes/uploaded" -name "IoT_Remote_Lab*.ttt" -type f -exec rm -f {} + 2>/dev/null
    find "$BASE_PATH/Assets/coppeliasim/scenes/uploaded" -name "test_*.ttt" -type f -exec rm -f {} + 2>/dev/null
    echo "  ✓ CoppeliaSim scenes cleaned"
fi

echo ""
echo "✓ Cleanup complete!"

