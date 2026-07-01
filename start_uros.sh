#!/bin/bash
set -e

DEV="/dev/ttyACM0"
CONTAINER="loki-uros"

docker rm -f "$CONTAINER" 2>/dev/null || true

if [ ! -e "$DEV" ]; then
  echo "ERROR: $DEV not found."
  exit 1
fi

echo "Resetting ESP32..."
python3 -c "
import serial, time
s = serial.Serial('$DEV', 115200)
s.dtr = False; time.sleep(0.1)
s.dtr = True;  time.sleep(1.0)
s.close()
"

echo "Starting micro-ROS agent..."
docker run -d --rm --name "$CONTAINER" \
  --network host \
  --device "$DEV:$DEV" \
  microros/micro-ros-agent:jazzy \
  serial --dev "$DEV" -b 115200

echo "micro-ROS agent started (container: $CONTAINER)."
