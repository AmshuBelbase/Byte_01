#!/usr/bin/env python3

import pickle
import socket
import time
from robot_config import SOCKET_HOST, SOCKET_PORT

# ── CHANGE THIS TO TEST A MOTOR ──────────────────────────────────────────────
TEST_MOTOR_ID = 8         # e.g., Motor 8 (Front-Right Hip Pitch)
TARGET_ANGLE = 15.0       # Move it 15 degrees mathematically (+ or -)
# ────────────────────────────────────────────────────────────────────────────

HOLD_SECONDS = 3.0

# Initialize all 12 motors to mathematical 0.0 (sitting position)
payload = {mid: 0.0 for mid in range(1, 13)}

# Override our test motor
payload[TEST_MOTOR_ID] = TARGET_ANGLE
payload["speed"] = 30.0  # Very slow, safe speed for testing 

data = pickle.dumps(payload)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((SOCKET_HOST, SOCKET_PORT))
    s.sendall(data)

print(f"Sent command: Motor {TEST_MOTOR_ID} -> {TARGET_ANGLE}°")
print(f"Holding for {HOLD_SECONDS}s... Watch the physical direction carefully.")
time.sleep(HOLD_SECONDS)

# Automatically return to 0.0 for safety after the test
payload[TEST_MOTOR_ID] = 0.0
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((SOCKET_HOST, SOCKET_PORT))
    s.sendall(pickle.dumps(payload))

print("Returned to zero.")