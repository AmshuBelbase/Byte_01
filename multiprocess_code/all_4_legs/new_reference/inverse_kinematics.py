#!/usr/bin/env python3
import math
import numpy as np
# IMPORT SHARED CONFIG
from robot_config import SIT_COORDS, LEG_ORDER


# ─── Kinematics Class ────────────────────────────────────────────────────────
class RoboticLeg:
    def __init__(self):
        # Updated specific link lengths
        self.L1 = 5.995
        self.Lc = 9.094
        self.L2 = 22.0
        self.L3 = 21.5

    def forward_kinematics(self, t1, t2, t3):
        # t1, t2, t3 in radians
        Le = self.L2 * math.cos(t2) + self.L3 * math.cos(t2 + t3)
        Lp = self.L2 * math.sin(t2) + self.L3 * math.sin(t2 + t3)

        X = -self.L1 - Le
        Y = self.Lc * math.cos(t1) + Lp * math.sin(t1)
        Z = -self.Lc * math.sin(t1) + Lp * math.cos(t1)

        return X, Y, Z

    def inverse_kinematics(self, X, Y, Z):
        # --- AUTO-DETECT LEG CONFIGURATION ---
        # If Y is negative, it's a Left Leg. If positive, it's a Right Leg.
        if Y < 0:
            shoulder_dir = -1
            knee_dir = 1
        else:
            shoulder_dir = 1
            knee_dir = -1

        # 1. Calculate Backward Extension (Le)
        Le = -X - self.L1

        # 2. Check Cylinder Intersection & Calculate Theta 1
        R_sq = Y**2 + Z**2
        if R_sq < self.Lc**2:
            raise ValueError("Reachability Error: Target is inside the Lc offset cylinder.")

        R = math.sqrt(R_sq)
        alpha = math.atan2(-Z, Y)

        # Uses the auto-detected shoulder_dir
        t1 = alpha - shoulder_dir * math.acos(self.Lc / R)
        Lp = Y * math.sin(t1) + Z * math.cos(t1)

        # 3. Check Workspace Distance Error
        D_sq = Le**2 + Lp**2
        cos_t3 = (D_sq - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)

        if cos_t3 < -1.0001 or cos_t3 > 1.0001:
            raise ValueError("Reachability Error: Target is physically out of bounds.")
        cos_t3 = max(-1.0, min(1.0, cos_t3)) 

        # 4. Calculate Theta 3 and Theta 2
        # Uses the auto-detected knee_dir
        t3 = knee_dir * math.acos(cos_t3)
        t2 = math.atan2(Lp, Le) - math.atan2(self.L3 * math.sin(t3), self.L2 + self.L3 * math.cos(t3))

        return math.degrees(t1), math.degrees(t2), math.degrees(t3)


# Instantiate the solver globally for the script to use
leg_solver = RoboticLeg()

# Flip and gear_ratio are handled downstream via motor_config.json — not here.
# Right legs and left legs can differ due to mechanical assembly variations.

# Leg IDs: front-left, back-left, front-right, back-right
legs = ["fl", "bl", "fr", "br"]


# ─── IK to Motor Angles Conversion ─────────────────────────────────────────────
def ik_to_motor_deg(t1, t2, t3, leg):
    """
    Convert IK angles to relative motor angles in degrees, applying per-leg sit co-ords offset.

    Args:
        t1, t2, t3: target angles of foot with respect to hip in DEGREES.
        leg: which leg ("fl", "bl", "fr", "br")

    Returns:
        (m1, m2, m3) relative motor angles in degrees

    Raises:
        ValueError if leg is invalid
    """
    if leg not in LEG_ORDER:
        raise ValueError(f"Invalid leg '{leg}'. Must be one of {LEG_ORDER}.")

    # Get the "zero position" angles for this specific leg's sitting posture
    c1, c2, c3 = leg_solver.inverse_kinematics(*SIT_COORDS[leg])

    # Calculate the delta (t1, c1, etc. are already in degrees from the class)
    m1 = t1 - c1
    m2 = t2 - c2
    m3 = t3 - c3

    return m1, m2, m3


# ─── Main IK Function for 1 Leg ─────────────────────────────────────────────
def calculate_each_motor_angles(x, y, z, leg):
    """
    Given a target foot position (x, y, z) in cm and leg ID, compute the motor angles to get there.
    Returns:
        Tuple of (m1, m2, m3) relative motor angles in degrees for the leg.
    """
    # compute target angles for foot position (returns degrees)
    t1, t2, t3 = leg_solver.inverse_kinematics(x, y, z) 
    
    # convert to relative motor angles
    m1, m2, m3 = ik_to_motor_deg(t1, t2, t3, leg) 
    
    return (m1, m2, m3)


# ─── IK for All Legs ─────────────────────────────────────────────────────────
def calculate_motor_angles(coords_dict):
    """
    Parameters:
    coords_dict: dict of leg_id -> (x, y, z) target foot positions in cm, where leg_id is one of "fl", "bl", "fr", "br".

    Given a dict of all 4 legs such that leg_id -> (x, y, z) target foot positions in cm, compute the motor angles to get there.
    
    Sample dict: 
    coords_dict = {
        "fl": (-9.094, 5.0, -30.0),
        "bl": (-9.094, -5.0, -30.0),
        "fr": (-9.094, 5.0, -30.0),
        "br": (-9.094, -5.0, -30.0)
    } 

    Returns:
        Dict of all 4 legs such that leg_id -> (m1, m2, m3) tuple of relative motor angles in degrees.
    """
    motor_angles = {}
    for leg, coords in coords_dict.items():
        motor_angles[leg] = calculate_each_motor_angles(*coords, leg)
    return motor_angles


# ─── Entry Point (testing only) ──────────────────────────────────────────────
if __name__ == "__main__":
    # First test: compute motor angles for target foot position for front-left leg only
    # Note: Passing the exact sit_coords should yield exactly (0.0, 0.0, 0.0)
    target_coords = sit_coords["fl"]
    motor_angles = calculate_each_motor_angles(*target_coords, leg="fl")
    print(f"Front-left leg motor angles (degrees) = {motor_angles}")

    # Second test: compute motor angles for target foot position for all 4 legs
    target_dict = {
        "fl": (-9.094, 5.0, -30.0),
        "bl": (-9.094, -5.0, -30.0),
        "fr": (-9.094, 5.0, -30.0),
        "br": (-9.094, -5.0, -30.0)
    }
    
    print("\nCalculating for all legs:")
    all_motor_angles = calculate_motor_angles(target_dict)
    for leg, angles in all_motor_angles.items():
        print(f"Leg {leg.upper()}: Motor Angles (degrees) = ({angles[0]:.2f}, {angles[1]:.2f}, {angles[2]:.2f})")