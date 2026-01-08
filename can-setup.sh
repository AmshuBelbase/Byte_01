#!/bin/bash
# CAN Interface Setup for Waveshare HAT on Ubuntu 22.04
# Motor requires 1Mbps bitrate

echo "=== Setting up CAN0 interface ==="

# Bring down if already running
sudo ip link set can0 down 2>/dev/null

# Configure CAN at 1Mbps (required by AK60-6 V3.0)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000

# Bring up the interface
sudo ip link set can0 up

# Verify
if ip link show can0 | grep -q "UP"; then
    echo "✓ CAN0 is UP and ready"
    ip -details link show can0 | grep -E "can|bitrate"
else
    echo "✗ Failed to bring up CAN0"
    exit 1
fi
