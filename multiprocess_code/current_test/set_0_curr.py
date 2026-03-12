#!/usr/bin/env python3
"""
AK60 — MOVE TO ANGLE, AUTO-ZERO ON ARRIVAL
Zeros when: current < 0.5A (settled) OR destination reached
"""

import numpy as np
import time
import can
from ak60_v3_control import AK60V3Motor

# ── Configuration ──────────────────────────────────────────
MOTOR_ID      = 13
CAN_INTERFACE = 'can0'

TARGET_DEG    = 90.0    # Angle to rotate to
KP            = 150.0
KD            = 1.5

LOOP_HZ       = 200
LOOP_DT       = 1.0 / LOOP_HZ

MIN_MOVE_TIME = 5.0     # Minimum move duration (s)
DEG_PER_SEC   = 0.05    # s/deg — increase to slow down

SETTLE_CURRENT = 0.5    # Amps — zero is set when current RISES TO or ABOVE this
SETTLE_CONFIRM = 0.3    # Seconds current must stay above threshold to confirm



class AutoZeroController:
    def __init__(self):
        self.bus   = None
        self.motor = None

        self.position    = 0.0
        self.current     = 0.0
        self.temperature = 0.0

        # S-curve state
        self.src_deg        = 0.0
        self.dest_deg       = TARGET_DEG
        self.movetime       = max(MIN_MOVE_TIME, abs(TARGET_DEG) * DEG_PER_SEC)
        self.move_start     = None
        self.move_active    = False
        self.move_done      = False

        # Settle tracking
        self.below_thresh_since = None   # time when current first went below threshold

        print(f"✓ Ready | Target: {TARGET_DEG}° | Move time: {self.movetime:.1f}s")
        print(f"  Will zero when current < {SETTLE_CURRENT}A for {SETTLE_CONFIRM}s  OR  destination reached")

    # ── Init ───────────────────────────────────────────────
    def init(self):
        print(f"🔌 CAN {CAN_INTERFACE} init...")
        self.bus   = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan', bitrate=1_000_000)
        self.motor = AK60V3Motor(MOTOR_ID, CAN_INTERFACE, bus=self.bus)
        self.motor.enable()
        time.sleep(0.3)
        self.motor.set_zero_position()
        print(f"  M{MOTOR_ID} ✓ — temporary zero set at boot position")

    # ── Feedback ───────────────────────────────────────────
    def read_feedback(self):
        if self.motor.read_feedback(timeout=0.003):
            self.position    = np.degrees(self.motor.position)
            self.current     = abs(self.motor.current)   # magnitude only
            self.temperature = self.motor.temperature

    # ── S-curve ────────────────────────────────────────────
    def get_eased_position(self):
        if not self.move_active or self.move_start is None:
            return np.radians(self.dest_deg)

        elapsed = time.time() - self.move_start
        if elapsed >= self.movetime:
            self.move_active = False
            return np.radians(self.dest_deg)

        s    = elapsed / self.movetime
        ease = 3*s*s - 2*s*s*s
        pos  = self.src_deg + ease * (self.dest_deg - self.src_deg)
        return np.radians(pos)

    def start_move(self):
        self.src_deg    = self.position
        self.move_start = time.time()
        self.move_active = True
        print(f"\n🚀 S-CURVE: {self.src_deg:.1f}° → {self.dest_deg:.1f}° ({self.movetime:.1f}s)")

    def check_and_set_zero(self):
        now = time.time()

        # Condition 1: current has been AT OR ABOVE threshold for sustained period
        if self.current >= SETTLE_CURRENT:
            if self.below_thresh_since is None:
                self.below_thresh_since = now
            elif now - self.below_thresh_since >= SETTLE_CONFIRM:
                print(f"\n✅ CONDITION 1: Current {self.current:.3f}A >= {SETTLE_CURRENT}A "
                    f"for {SETTLE_CONFIRM}s → Setting zero at {self.position:.2f}°")
                self.motor.set_zero_position()
                return True
        else:
            self.below_thresh_since = None  # reset if current drops back down

        # Condition 2: destination reached (move finished, position close)
        if not self.move_active and abs(self.position - self.dest_deg) < 2.0:
            print(f"\n✅ CONDITION 2: Reached {self.position:.2f}° (target {self.dest_deg:.1f}°) "
                f"→ Setting zero")
            self.motor.set_zero_position()
            return True

        return False

    
    # ── Main Loop ──────────────────────────────────────────
    def run(self):
        self.init()

        # Short settle before moving
        time.sleep(0.5)
        self.start_move()

        last_print = 0.0
        iteration  = 0

        print("🔄 Running — will auto-stop after zero is set\n")
        try:
            while True:
                iteration += 1
                loop_start = time.time()

                self.read_feedback()

                pos_rad = self.get_eased_position()
                self.motor.send_mit_command(
                    position=pos_rad,
                    velocity=0.0,
                    kp=min(450, KP),
                    kd=KD,
                    torque=0.0,
                )

                # Check zero conditions
                zeroed = self.check_and_set_zero()

                # Status print
                now = time.time()
                if now - last_print > 0.3:
                    last_print = now
                    state = "🚀 MOVING" if self.move_active else "⏸️  HOLDING"
                    print(
                        f'\r[{iteration:6d}] {state} | '
                        f'Pos: {self.position:6.2f}°/{self.dest_deg:.1f}° | '
                        f'Curr: {self.current:+.3f}A | '
                        f'Temp: {self.temperature:.1f}°C',
                        end='', flush=True
                    )

                if zeroed:
                    print(f"\n🏁 New zero set. Motor will hold at 0° (new origin).")
                    # Hold at new zero for 2s so motor settles, then exit
                    for _ in range(int(2.0 / LOOP_DT)):
                        self.motor.send_mit_command(0.0, 0.0, KP, KD, 0.0)
                        time.sleep(LOOP_DT)
                    break

                elapsed = time.time() - loop_start
                time.sleep(max(0.0, LOOP_DT - elapsed))

        except KeyboardInterrupt:
            print("\n🛑 Interrupted")
        finally:
            self.cleanup()

    # ── Cleanup ────────────────────────────────────────────
    def cleanup(self):
        print("🧹 Cleanup...")
        try:
            self.motor.send_mit_command(0, 0, 0, 5, 0)
            time.sleep(0.1)
            self.motor.disable()
        except Exception:
            pass
        if self.bus:
            self.bus.shutdown()
        print("✅ Done!")


if __name__ == "__main__":
    print("=" * 55)
    print("🎮  AK60  |  MOVE + AUTO-ZERO ON SETTLE")
    print("=" * 55)
    AutoZeroController().run()
