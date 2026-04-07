#!/bin/bash

# Enable error checking
set -e

# Logging functions
log_info() {
    echo "[INFO] $1"
}

log_error() {
    echo "[ERROR] $1" >&2
}

# Check for parameters
if [ -z "$1" ]; then
    log_error "Usage: $0 <ROS2_BRIDGE_DIR>"
    exit 1
fi

KUAVO_ROS2_BRIDGE_DIR=$1

# Set directory paths
SCRIPT_DIR="$(pwd)"
KUAVO_ROS_CONTROL_DIR="$(cd "$(dirname "$SCRIPT_DIR")" && pwd)"
KUAVO_MSGS_DIR="$KUAVO_ROS_CONTROL_DIR/src/kuavo_msgs"


log_info "KUAVO_ROS_CONTROL_DIR: $KUAVO_ROS_CONTROL_DIR"
log_info "KUAVO_MSGS_DIR: $KUAVO_MSGS_DIR"
log_info "KUAVO_ROS2_BRIDGE_DIR: $KUAVO_ROS2_BRIDGE_DIR"

# Create target directory if it does not exist
mkdir -p "$KUAVO_ROS2_BRIDGE_DIR"

# Function to sync ROS projects (excluding CMake files)
sync_ros_project() {
    local src_dir=$1
    local dst_dir=$2
    local -a extra_excludes=("${@:3}")  # Capture all remaining arguments as array
    local project_name=$(basename "$src_dir")
    
    log_info "Syncing ROS project $project_name..."
    
    # Create target directory
    mkdir -p "$dst_dir/$project_name"
    
    # Build base exclude options
    local exclude_opts=(
        --exclude 'build/'
        --exclude 'devel/'
        --exclude 'install/'
        --exclude 'log/'
        --exclude '.git/'
        --exclude 'CMakeLists.txt'
        --exclude 'package.xml'
        --exclude '*.cmake'
    )
    
    # Add extra excludes if provided
    for item in "${extra_excludes[@]}"; do
        exclude_opts+=(--exclude "$item")
    done
    
    # Use rsync to sync files
    if rsync -av --delete "${exclude_opts[@]}" \
        --stats \
        "$src_dir/" "$dst_dir/$project_name/"; then
        log_info "Successfully synced $project_name"
    else
        log_error "Failed to sync $project_name"
        exit 1
    fi
}

mkdir -p "$KUAVO_ROS2_BRIDGE_DIR/ros1/src"
rm -rf "$KUAVO_ROS2_BRIDGE_DIR/ros1/src/kuavo_msgs" || true
cp -r "$KUAVO_MSGS_DIR" "$KUAVO_ROS2_BRIDGE_DIR/ros1/src"

mkdir -p "$KUAVO_ROS2_BRIDGE_DIR/ros2/src"
rm -rf "$KUAVO_ROS2_BRIDGE_DIR/ros2/src/kuavo_msgs/msg" || true
rm -rf "$KUAVO_ROS2_BRIDGE_DIR/ros2/src/kuavo_msgs/srv" || true
sync_ros_project "$KUAVO_MSGS_DIR" "$KUAVO_ROS2_BRIDGE_DIR/ros2/src"

log_info "All projects synced successfully!"

cd "$KUAVO_ROS2_BRIDGE_DIR/ros2/src/kuavo_msgs"

python3 format_msgs.py

python3 generate_mapping_rules.py --ros1_path "$KUAVO_ROS2_BRIDGE_DIR/ros1/src/kuavo_msgs" --ros2_path "$KUAVO_ROS2_BRIDGE_DIR/ros2/src/kuavo_msgs"

log_info "Format the ROS1 messages to ROS2 messages"