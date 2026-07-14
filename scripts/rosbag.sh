#!/usr/bin/env bash
set -e

BAG_DIR="$HOME/ros_bags"
mkdir -p "$BAG_DIR"
cd "$BAG_DIR"

BAG_NAME="ros_bag_$(date +%Y%m%d_%H%M%S)"

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
  /diagnostics \
  /system/arm_state.data \
  /pc_to_esp_cmd \
  /monitor/target/depth.data \
  /monitor/depth.data \
  /monitor/target/heading.data \
  /monitor/heading_deg.data \
  /monitor/target/speed.data \
  /monitor/speed.data \
  /pc_to_esp_cmd \
  /vesc1/vesc/telemetry/current_in.data \
  /vesc1/vesc/telemetry/voltage_in.data \
  /vesc1/vesc/telemetry/rpm.data \
  /vesc1/vesc/telemetry/duty.data \
  /vesc1/vesc/telemetry/fault.data
