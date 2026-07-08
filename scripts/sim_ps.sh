#!/usr/bin/env bash
LAUNCH_PATTERN='ros2 launch loki_bringup sim.launch.py'
NODE_PATTERNS=(
    'ros_tcp_endpoint/default_server_endpoint'
    'robot_localization/ekf_node'
    'loki_control/lib/loki_control/loki_controller'
    'loki_monitor/lib/loki_monitor/monitor'
    'foxglove_bridge/foxglove_bridge'
)

echo "── sim stack processes ──────────────────────────────"
launch_pid=$(pgrep -f "$LAUNCH_PATTERN")
[ -n "$launch_pid" ] && echo "  launch  PID $launch_pid" || echo "  launch  not running"
for pat in "${NODE_PATTERNS[@]}"; do
    for pid in $(pgrep -f "$pat"); do
        echo "  node    PID $pid: ${pat##*/}"
    done
done

echo "── port :10000 (ros_tcp_endpoint) ───────────────────"
holder=$(ss -tlnp 2>/dev/null | grep ':10000')
if [ -n "$holder" ]; then
    echo "  $holder"
else
    echo "  free"
fi

if [ "$1" = "--kill" ]; then
    echo "── killing ──────────────────────────────────────────"
    if [ -n "$launch_pid" ]; then
        echo "  SIGINT -> launch $launch_pid (cascades to nodes)"
        kill -INT $launch_pid 2>/dev/null
        sleep 2
    fi
    still=$(for pat in "${NODE_PATTERNS[@]}"; do pgrep -f "$pat"; done | sort -u)
    if [ -n "$still" ]; then
        echo "  stragglers survived, SIGKILL -> $(echo $still)"
        kill -9 $still 2>/dev/null
        sleep 1
    fi
    if ss -tlnp 2>/dev/null | grep -q ':10000'; then
        echo "  WARNING: :10000 still bound — a process outside the sim tree holds it"
    else
        echo "  :10000 free"
    fi
    echo "  done"
fi
