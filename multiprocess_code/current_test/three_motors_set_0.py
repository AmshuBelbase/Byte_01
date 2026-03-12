#!/usr/bin/env python3
"""
AK60 — SEQUENTIAL 3-MOTOR MOVE + AUTO-ZERO + POST-ZERO NUDGE
After each motor sets its permanent zero, it rotates by nudge_deg from that new zero.
"""

import numpy as np
import time
import can
from ak60_v3_control import AK60V3Motor

# ── Configuration ──────────────────────────────────────────
CAN_INTERFACE = 'can1'

# (motor_id, first_target_deg, step_deg, trigger_current_A, nudge_deg)
MOTOR_SEQUENCE = [
    (8,   60.0,  10.0,  3.0,  -20.0),
    (9,  -10.0, -10.0,  5.0,   20.0),
    (7,  -60.0, -10.0,  4.0,   0.0),
]

KP = 150.0
KD = 1.5

LOOP_HZ       = 200
LOOP_DT       = 1.0 / LOOP_HZ

MIN_MOVE_TIME = 1.0
DEG_PER_SEC   = 0.0005

TRIGGER_CONFIRM = 0.3  # s


class MotorUnit:
    def __init__(self, bus, motor_id, first_target, step_deg, trigger_current, nudge_deg):
        self.bus             = bus
        self.motor_id        = motor_id
        self.step_deg        = step_deg
        self.trigger_current = trigger_current
        self.nudge_deg       = nudge_deg            # degrees to rotate after zero is set
        self.hold_deg        = 0.0                  # position to hold after everything is done
        self.motor           = AK60V3Motor(motor_id, CAN_INTERFACE, bus=bus)

        self.position    = 0.0
        self.current     = 0.0
        self.temperature = 0.0

        self.src_deg     = 0.0
        self.dest_deg    = first_target
        self.movetime    = max(MIN_MOVE_TIME, abs(first_target) * DEG_PER_SEC)
        self.move_start  = None
        self.move_active = False

        self.above_thresh_since = None
        self.zeroed      = False
        self.nudge_done  = False    # True once nudge move completes

    # ── Init ───────────────────────────────────────────────
    def init(self):
        self.motor.enable()
        time.sleep(0.3)
        self.motor.set_zero_position()
        print(f"  M{self.motor_id} ✓ enabled | trigger: {self.trigger_current}A | nudge: {self.nudge_deg:+.1f}°")

    # ── Feedback ───────────────────────────────────────────
    def read_feedback(self):
        if self.motor.read_feedback(timeout=0.003):
            self.position    = np.degrees(self.motor.position)
            self.current     = abs(self.motor.current)
            self.temperature = self.motor.temperature

    # ── S-curve ────────────────────────────────────────────
    def get_eased_position(self):
        if not self.move_active or self.move_start is None:
            return np.radians(self.dest_deg)

        elapsed = time.time() - self.move_start
        if elapsed >= self.movetime:
            self.move_active = False
            self.src_deg = self.dest_deg
            return np.radians(self.dest_deg)

        s    = elapsed / self.movetime
        ease = 3*s*s - 2*s*s*s
        pos  = self.src_deg + ease * (self.dest_deg - self.src_deg)
        return np.radians(pos)

    def start_move(self, from_deg, to_deg):
        self.src_deg     = from_deg
        self.dest_deg    = to_deg
        self.movetime    = max(MIN_MOVE_TIME, abs(to_deg - from_deg) * DEG_PER_SEC)
        self.move_start  = time.time()
        self.move_active = True
        print(f"\n🚀 M{self.motor_id}: {from_deg:.1f}° → {to_deg:.1f}° ({self.movetime:.1f}s)")

    # ── Commands ───────────────────────────────────────────
    def send_command(self):
        pos_rad = self.get_eased_position()
        self.motor.send_mit_command(
            position=pos_rad,
            velocity=0.0,
            kp=min(450, KP),
            kd=KD,
            torque=0.0,
        )

    def hold_at_zero(self):
        """Hold at final resting position (nudge_deg from new zero)."""
        self.motor.send_mit_command(
            np.radians(self.hold_deg), 0.0, KP, KD, 0.0
        )

    def hold_at_boot_zero(self):
        self.motor.send_mit_command(0.0, 0.0, KP, KD, 0.0)

    # ── Current trigger ────────────────────────────────────
    def check_current_trigger(self):
        now = time.time()
        if self.current >= self.trigger_current:
            if self.above_thresh_since is None:
                self.above_thresh_since = now
            elif now - self.above_thresh_since >= TRIGGER_CONFIRM:
                return True
        else:
            self.above_thresh_since = None
        return False

    # ── Nudge: start move from new zero to nudge_deg ───────
    def start_nudge(self):
        """Called immediately after set_zero_position(). Moves to nudge_deg from new zero."""
        print(f"\n↩️  M{self.motor_id} nudge: 0° → {self.nudge_deg:+.1f}° (from new zero)")
        self.start_move(0.0, self.nudge_deg)

    # ── Disable ────────────────────────────────────────────
    def disable(self):
        try:
            self.motor.send_mit_command(0, 0, 0, 5, 0)
            time.sleep(0.1)
            self.motor.disable()
            print(f"  M{self.motor_id} disabled ✓")
        except Exception:
            pass


# ── Main ───────────────────────────────────────────────────
def main():
    print("=" * 60)
    print("🎮  AK60  |  ALL MOTORS ACTIVE  |  AUTO-ZERO + NUDGE")
    for mid, first, step, trig, nudge in MOTOR_SEQUENCE:
        print(f"   M{mid}: step {step:+.0f}° | trigger {trig}A | nudge {nudge:+.1f}°")
    print(f"   Confirm: {TRIGGER_CONFIRM}s")
    print("=" * 60)

    bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan', bitrate=1_000_000)

    print("\n🔌 Enabling all motors...")
    units = {}
    for motor_id, first_target, step_deg, trigger_current, nudge_deg in MOTOR_SEQUENCE:
        u = MotorUnit(bus, motor_id, first_target, step_deg, trigger_current, nudge_deg)
        u.init()
        units[motor_id] = u
    print("✅ All motors enabled\n")

    sequence_ids = [mid for mid, *_ in MOTOR_SEQUENCE]
    active_idx   = 0
    zeroed_ids   = set()      # zeroed AND nudge complete
    nudging_ids  = set()      # zeroed but still doing nudge move

    last_print = 0.0
    iteration  = 0

    active_id = sequence_ids[active_idx]
    units[active_id].start_move(units[active_id].position, units[active_id].dest_deg)
    print(f"[STEP 1/{len(sequence_ids)}] Running M{active_id}...")

    try:
        while True:
            iteration += 1
            loop_start = time.time()

            # Read ALL
            for u in units.values():
                u.read_feedback()

            # Command ALL
            for motor_id, u in units.items():
                if motor_id in zeroed_ids:
                    u.hold_at_zero()                   # fully done → hold at nudge position
                elif motor_id in nudging_ids:
                    u.send_command()                   # doing nudge move → follow S-curve
                elif motor_id == sequence_ids[active_idx] if active_idx < len(sequence_ids) else False:
                    u.send_command()                   # active zeroing motor → follow S-curve
                else:
                    u.hold_at_boot_zero()              # waiting → hold at boot zero

            # ── Nudge completion check ─────────────────────
            for motor_id in list(nudging_ids):
                u = units[motor_id]
                if not u.move_active:
                    u.hold_deg = u.nudge_deg    # ← hold at nudge position, not 0
                    print(f"\n✅ M{motor_id} nudge complete — holding at {u.dest_deg:+.1f}° from new zero")
                    nudging_ids.discard(motor_id)
                    zeroed_ids.add(motor_id)

            # Active motor logic
            if active_idx < len(sequence_ids):
                active_id = sequence_ids[active_idx]
                active_u  = units[active_id]

                triggered = active_u.check_current_trigger()

                # Reached dest without trigger → chain next step ONLY if current still safe
                if not active_u.move_active and not triggered \
                        and active_u.current < active_u.trigger_current \
                        and active_id not in zeroed_ids and active_id not in nudging_ids:
                    next_target = active_u.dest_deg + active_u.step_deg
                    print(f"\n📍 M{active_id}: Reached {active_u.dest_deg:.1f}° — "
                        f"no trigger, chaining to {next_target:.1f}°")
                    active_u.start_move(active_u.dest_deg, next_target)   # ← no timer reset

                # Trigger fired → set zero → nudge or skip → hand off
                if triggered:
                    print(f"\n✅ M{active_id} TRIGGERED: "
                        f"{active_u.current:.3f}A >= {active_u.trigger_current}A "
                        f"→ Setting zero at {active_u.position:.2f}°")
                    active_u.motor.set_zero_position()
                    active_u.zeroed = True
                    active_idx += 1

                    if active_u.nudge_deg != 0.0:
                        nudging_ids.add(active_id)
                        active_u.start_nudge()
                    else:
                        active_u.hold_deg = 0.0
                        zeroed_ids.add(active_id)
                        print(f"  M{active_id} nudge=0° — holding at new zero")

                    if active_idx < len(sequence_ids):
                        next_id = sequence_ids[active_idx]
                        print(f"\n[STEP {active_idx+1}/{len(sequence_ids)}] Starting M{next_id}...")
                        units[next_id].start_move(
                            units[next_id].position,
                            units[next_id].dest_deg
                        )
                    else:
                        print("\n" + "="*60)
                        print("🏁 ALL MOTORS ZEROED — Holding all. Ctrl+C to exit.")
                        print("="*60)


            # All nudges done check
            if active_idx >= len(sequence_ids) and not nudging_ids:
                pass  # just hold forever — status print handles this

            # Status
            now = time.time()
            if now - last_print > 0.3:
                last_print = now
                parts = []
                for motor_id in sequence_ids:
                    u = units[motor_id]
                    if motor_id in zeroed_ids:
                        icon = "✅"
                    elif motor_id in nudging_ids:
                        icon = "↩️ "
                    elif motor_id == sequence_ids[active_idx] if active_idx < len(sequence_ids) else False:
                        icon = "🚀" if u.move_active else "⏸️ "
                    else:
                        icon = "💤"
                    parts.append(
                        f"{icon}M{motor_id}: {u.position:6.2f}°→{u.dest_deg:+.0f}° "
                        f"| {u.current:.3f}A/{u.trigger_current}A"
                    )
                print(f'\r[{iteration:6d}]  ' + '  |  '.join(parts), end='', flush=True)

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, LOOP_DT - elapsed))

    except KeyboardInterrupt:
        print("\n🛑 Ctrl+C — releasing all motors...")

    finally:
        for u in units.values():
            u.disable()
        bus.shutdown()
        print("✅ Done!")


if __name__ == "__main__":
    main()