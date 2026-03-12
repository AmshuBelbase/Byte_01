#!/usr/bin/env python3
"""
AK60 S-CURVE MOTOR CONTROL — NO PLOTS, MAX SMOOTHNESS
200Hz loop | Slow S-curve | Clean feedback read
"""

import numpy as np
import time
import json
import threading
import can
from ak60_v3_control import AK60V3Motor

# ── Configuration ──────────────────────────────────────────
motor_ids     = [13]
num_motors    = len(motor_ids)
can_interface = 'can0'

LOOP_HZ       = 200          # Control loop rate
LOOP_DT       = 1.0 / LOOP_HZ

MIN_MOVE_TIME = 1.0          # Minimum seconds for any move
DEG_PER_SEC   = 0.000001         # s/deg → lower = faster (increase for smoother/slower)
VFF_SCALE     = 1.0          # Velocity feedforward multiplier (0.0 to disable, 1.0 full)

M_LOAD = np.array([0])      # kg
R_LOAD = np.array([0])      # m


class MotorController:
    def __init__(self):
        self.motors      = {}
        self.shared_bus  = None

        self.positions       = np.zeros(num_motors)
        self.current_values  = np.zeros(num_motors)
        self.temperatures    = np.zeros(num_motors)

        self.target_angles   = np.zeros(num_motors)
        self.kp_gains        = np.full(num_motors, 150.0)
        self.kd_gains        = np.full(num_motors, 1.5)

        # S-curve state per motor
        self.move_active      = [False] * num_motors
        self.movetime         = np.full(num_motors, MIN_MOVE_TIME)
        self.src_deg          = np.zeros(num_motors)
        self.dest_deg         = np.zeros(num_motors)
        self.movestart_times  = [None] * num_motors

        self.start_time  = time.time()
        self.iteration   = 0
        print("✓ Motor controller ready — no plots, max smoothness")

    # ── CAN / Motor Init ───────────────────────────────────
    def init_can_and_motors(self):
        print(f"🔌 CAN {can_interface} init...")
        self.shared_bus = can.interface.Bus(
            channel=can_interface, bustype='socketcan', bitrate=1_000_000
        )
        for mid in motor_ids:
            motor = AK60V3Motor(mid, can_interface, bus=self.shared_bus)
            motor.enable()
            time.sleep(0.2)
            motor.set_zero_position()
            self.motors[mid] = motor
            print(f"  M{mid} ✓")
        print("✅ Motors ready")

    # ── JSON Config ────────────────────────────────────────
    def read_json_config(self):
        try:
            with open('motor_config.json', 'r') as f:
                config = json.load(f)
            changed = False
            for i, mid in enumerate(motor_ids):
                new_angle = float(config['motors'][str(mid)]['angle'])
                new_kp    = float(config['motors'][str(mid)]['kp'])
                new_kd    = float(config['motors'][str(mid)]['kd'])
                if (abs(new_angle - self.target_angles[i]) > 0.1 or
                        abs(new_kp - self.kp_gains[i]) > 1 or
                        abs(new_kd - self.kd_gains[i]) > 0.1):
                    changed = True
                self.target_angles[i] = new_angle
                self.kp_gains[i]      = new_kp
                self.kd_gains[i]      = new_kd
            return changed
        except Exception:
            return False

    # ── Feedback ───────────────────────────────────────────
    def read_feedback_all(self):
        """Read all motors once per loop tick."""
        for i, mid in enumerate(motor_ids):
            motor = self.motors[mid]
            if motor.read_feedback(timeout=0.003):
                self.positions[i]      = np.degrees(motor.position)
                self.current_values[i] = motor.current
                self.temperatures[i]   = motor.temperature

    # ── S-curve Logic ──────────────────────────────────────
    def check_and_start_move(self, i):
        if abs(self.target_angles[i] - self.dest_deg[i]) > 1.0:
            self.src_deg[i]  = self.positions[i]
            self.dest_deg[i] = self.target_angles[i]
            angle_diff       = abs(self.dest_deg[i] - self.src_deg[i])
            self.movetime[i] = max(MIN_MOVE_TIME, angle_diff * DEG_PER_SEC)
            self.movestart_times[i] = time.time()
            self.move_active[i]     = True
            print(f"\n🎯 M{motor_ids[i]} S-CURVE: "
                  f"{self.src_deg[i]:.1f}° → {self.dest_deg[i]:.1f}° "
                  f"({self.movetime[i]:.1f}s)")

    def get_control_target(self, i):
        if not self.move_active[i] or self.movestart_times[i] is None:
            return np.radians(self.target_angles[i]), 0.0

        elapsed = time.time() - self.movestart_times[i]
        if elapsed >= self.movetime[i]:
            self.move_active[i] = False
            return np.radians(self.target_angles[i]), 0.0

        s = max(0.0, min(1.0, elapsed / self.movetime[i]))

        # Hermite S-curve: smooth-step position
        pos_progress = 3*s*s - 2*s*s*s
        pos_deg      = self.src_deg[i] + pos_progress * (self.dest_deg[i] - self.src_deg[i])
        pos_rad      = np.radians(pos_deg)

        # Derivative: velocity feedforward
        vel_progress = (6*s - 6*s*s) / self.movetime[i]
        vel_rad_s    = np.radians((self.dest_deg[i] - self.src_deg[i]) * vel_progress) * VFF_SCALE

        return pos_rad, vel_rad_s

    # ── Main Control Loop ──────────────────────────────────
    def main_control_loop(self):
        last_print = 0.0
        print(f"🚀 Control loop running at {LOOP_HZ}Hz")
        try:
            while True:
                self.iteration += 1
                loop_start = time.time()

                config_changed = self.read_json_config()

                # Check for new move targets
                if config_changed:
                    for i in range(num_motors):
                        self.check_and_start_move(i)

                # Read feedback ONCE per tick (not inside motor loop)
                self.read_feedback_all()

                # Send commands
                for i in range(num_motors):
                    pos_rad, vel_rad = self.get_control_target(i)
                    tau_grav = M_LOAD[i] * 9.81 * R_LOAD[i] * np.sin(pos_rad)
                    self.motors[motor_ids[i]].send_mit_command(
                        position=pos_rad,
                        velocity=vel_rad,
                        kp=min(450, self.kp_gains[i]),
                        kd=self.kd_gains[i],
                        torque=tau_grav,
                    )

                # Status printout ~3Hz
                now = time.time()
                if now - last_print > 0.3:
                    last_print = now
                    parts = []
                    for i in range(num_motors):
                        icon = "🚀" if self.move_active[i] else "⏸️ "
                        parts.append(
                            f"{icon}M{motor_ids[i]}: "
                            f"{self.positions[i]:6.2f}°/{self.target_angles[i]:6.2f}° "
                            f"| {self.current_values[i]:+6.3f}A "
                            f"| {self.temperatures[i]:.1f}°C"
                        )
                    print(f'\r[{self.iteration:6d}]  ' + '  '.join(parts), end='', flush=True)

                # Pace to LOOP_HZ
                elapsed = time.time() - loop_start
                time.sleep(max(0.0, LOOP_DT - elapsed))

        except KeyboardInterrupt:
            print("\n🛑 KeyboardInterrupt — stopping")

    # ── Cleanup ────────────────────────────────────────────
    def cleanup(self):
        print("\n🧹 Cleanup...")
        for mid in motor_ids:
            try:
                self.motors[mid].send_mit_command(0, 0, 0, 5, 0)
                time.sleep(0.1)
                self.motors[mid].disable()
            except Exception:
                pass
        if self.shared_bus:
            self.shared_bus.shutdown()
        print("✅ Done!")

    # ── Entry Point ────────────────────────────────────────
    def run(self):
        self.init_can_and_motors()
        try:
            self.main_control_loop()
        finally:
            self.cleanup()


if __name__ == "__main__":
    print("=" * 55)
    print("🎮  AK60 S-CURVE CONTROL  |  NO PLOTS  |  200Hz")
    print("=" * 55)
    MotorController().run()
