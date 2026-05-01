# robot_config.py

# ─── Network Settings ────────────────────────────────────────────────────────
SOCKET_HOST = "10.196.200.34" #127.0.0.1
SOCKET_PORT = 50000

# ─── Leg Identifiers ─────────────────────────────────────────────────────────
LEG_ORDER = ("fl", "bl", "fr", "br")

# ─── Sitting Cartesian Coordinates (X, Y, Z in cm) ───────────────────────────
# Used by Inverse Kinematics and as the baseline for live socket tracking.
SIT_COORDS = {
    "fl": (-9.094, 5.0, -30.0),
    "bl": (-9.094, -5.0, -30.0),
    "fr": (-9.094, 5.0, -30.0),
    "br": (-9.094, -5.0, -30.0)
}

# ─── Physical Sitting Motor Angles (Degrees) ─────────────────────────────────
# The absolute hardware angles the motors must reach BEFORE zeroing.
SIT_TARGETS_DEG = {
    1: 85.0,  2: -114.0, 3: 6.0,    # fl
    4: 85.0,  5: -98.0,  6: 9.0,    # bl
    7: 85.0,  8: -114.0, 9: 4.0,    # fr
    10: 85.0, 11: -98.0, 12: 10.0   # br
}