#!/usr/bin/env bash
set -e

BAG_DIR="$HOME/auv_bags"
mkdir -p "$BAG_DIR"
cd "$BAG_DIR"

BAG_NAME="auv_bag_$(date +%Y%m%d_%H%M%S)"

echo "Recording ROS Bag: $BAG_DIR/$BAG_NAME"
echo "Press Ctrl+C to stop recording."

ros2 bag record -o "$BAG_NAME" \
  /esp/raw_sensor \
  /imu/data_raw \
  /imu/mag \
  /imu/data \
  /dvl/twist_stamped \
  /odometry/filtered \
  /tf \
  /tf_static \
  /diagnostics