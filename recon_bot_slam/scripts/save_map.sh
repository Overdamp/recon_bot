#!/bin/bash

# Default map name if not provided
MAP_NAME=${1:-my_map}
MAP_DIR=~/Ros2_Directory/recon_ws/src/recon_bot_slam/maps

# Create directory if it doesn't exist
mkdir -p $MAP_DIR

echo "Saving map as '$MAP_NAME' to $MAP_DIR..."

# Run the map saver
ros2 run nav2_map_server map_saver_cli -f $MAP_DIR/$MAP_NAME

if [ $? -eq 0 ]; then
    echo "Map saved successfully!"
    echo "Files created:"
    echo "  - $MAP_DIR/$MAP_NAME.yaml"
    echo "  - $MAP_DIR/$MAP_NAME.pgm"
else
    echo "Failed to save map. Make sure SLAM is running."
fi
