#!/usr/bin/env bash
# VESC process & port hygiene.
#
# Lists every running vesc_node, shows which serial device each one holds,
# and flags stale handles (process holding a tty that no longer exists —
# happens when a VESC drops off USB and re-enumerates under a new name).
# Also checks the /dev/vesc_* udev symlinks are present.
#
# Usage:
#   scripts/vesc_ps.sh          status only
#   scripts/vesc_ps.sh --kill   also stop all vesc_nodes (SIGINT first, so a
#                               live node zeroes its thruster on the way out)

# Matches only the actual node process (install/loki_vesc/lib/loki_vesc/vesc_node),
# not the `pixi run` / `ros2 run` wrappers, which exit with it.
PATTERN='loki_vesc/vesc_node'

pids=$(pgrep -f "$PATTERN")

echo "── vesc_node processes ──────────────────────────────"
if [ -z "$pids" ]; then
    echo "  none running"
else
    for pid in $pids; do
        tty=$(readlink "/proc/$pid/fd/"* 2>/dev/null | grep -m1 '^/dev/ttyACM')
        if [ -z "$tty" ]; then
            state="no serial port open (still connecting?)"
        elif [ -e "$tty" ]; then
            state="holding $tty  [OK]"
        else
            state="holding $tty which NO LONGER EXISTS  [STALE — kill this]"
        fi
        echo "  PID $pid: $state"
        echo "       $(tr '\0' ' ' < "/proc/$pid/cmdline")"
    done
fi

echo "── udev symlinks ────────────────────────────────────"
for link in /dev/vesc_1 /dev/vesc_2; do
    if [ -e "$link" ]; then
        echo "  $link -> $(readlink "$link")  [present]"
    else
        echo "  $link MISSING — VESC not enumerated on its pinned USB port"
    fi
done

if [ "$1" = "--kill" ] && [ -n "$pids" ]; then
    echo "── killing ──────────────────────────────────────────"
    echo "  SIGINT -> $pids (clean shutdown zeroes the thruster)"
    kill -INT $pids 2>/dev/null
    sleep 1
    still=$(pgrep -f "$PATTERN")
    if [ -n "$still" ]; then
        echo "  still alive, SIGKILL -> $still"
        kill -9 $still 2>/dev/null
    fi
    echo "  done"
fi
