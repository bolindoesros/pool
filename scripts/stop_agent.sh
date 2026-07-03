#!/bin/bash
CONTAINER="loki-uros"

if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo "Stopping micro-ROS agent ($CONTAINER)..."
    docker rm -f "$CONTAINER"
    echo "Stopped."
else
    echo "No container named $CONTAINER is running."
fi