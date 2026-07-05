#!/usr/bin/env bash
set -e

IFACE="enP8p1s0"
JETSON_IP="192.168.2.10/24"

echo "Configuring $IFACE as $JETSON_IP..."

sudo ip addr flush dev "$IFACE"
sudo ip addr add "$JETSON_IP" dev "$IFACE"
sudo ip link set "$IFACE" up

ip addr show "$IFACE"

echo "Ethernet configured for Tracker 650."