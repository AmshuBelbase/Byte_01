#!/usr/bin/env python3
"""
AK60 — 4-LEG SEQUENTIAL HOMING
12 motors | 2 CAN buses | 2-phase parallel homing
Phase 1: Front Right (can1: M8→M9→M7) + Back Left (can0: M5→M6→M4)
Phase 2: Back Right (can1: M11→M12→M10) + Front Left (can0: M2→M3→M1)
Final:   Single thread holds all 12 motors at nudge positions forever
"""

import numpy as np
import time
import threading
import can
from ak60_v3_control import AK60V3Motor

# ── Tuning ─────────────────────────────────────────────────
KP              = 150.0
KD              = 1.5
LOOP_HZ         = 200
LOOP_DT         = 1.0 / LOOP_HZ
MIN_MOVE_TIME   = 1.0
DEG_PER_SEC     = 0.0005
TRIGGER_CONFIRM = 0.3   # s

# ── Leg Configurations ─────────────────────────────────────
# (motor_id, first_target_deg, step_deg, trigger_current_A, nudge_deg)
CAN_CONFIG = {
    'can0': {
        'phase1': [          # Back Left leg: M5 → M6 → M4
            (5, -60.0, -10.0, 4.0,  20.0),
            (6,  10.0,  10.0, 5.0, -20.0),
            (4,  60.0,  10.0, 4.0, -60.0),
        ],
        'phase2': [          # Front Left leg: M2 → M3 → M1
            (2, -60.0, -10.0, 4.0,  20.0),
            (3,  10.0,  10.0, 5.0, -20.0),
            (1,  60.0,  10.0, 4.0, -60.0),
        ],
    },
    'can1': {
        'phase1': [          # Front Right leg: M8 → M9 → M7
            (8,  60.0,  10.0, 4.0, -20.0),
            (9, -10.0, -10.0, 5.0,  20.0),
            (7, -60.0, -10.0, 4.0,  60.0),
        ],
        'phase2': [          # Back Right leg: M11 → M12 → M10
            (11,  60.0,  10.0, 4.0, -20.0),
            (12, -10.0, -10.0, 5.0,  20.0),
            (10, -60.0, -10.0, 4.0,  60.0),
        ],
    },
}


# ── Motor Unit ─────────────────────────────────────────────
class MotorUnit:
    def __init__(self, bus, motor_id, can_interface, first_target, step_deg, trigger_current, nudge_deg):
        self.motor_id        = motor_id
        self.step_deg        = step_deg
        self.trigger_current = trigger_current
        self.nudge_deg       = nudge_deg
        self.hold_deg        = 0.0      # final hold position in new-zero frame
        self.motor           = AK60V3Motor(motor_id, can_interface, bus=bus)

        self.position    = 0.0
        self.current     = 0.0
        self.temperature = 0.0

        self.src_deg     = 0.0
        self.dest_deg    = first_target
        self.movetime    = max(MIN_MOVE_TIME, abs(first_target) * DEG_PER_SEC)
        self.move_start  = None
        self.move_active = False

        self.above_thresh_since = None
        self.zeroed             = False

    def init(self):
        self.motor.enable()
        time.sleep(0.3)
        self.motor.set_zero_position()

    def read_feedback(self):
        if self.motor.read_feedback(timeout=0.003):
            self.position    = np.degrees(self.motor.position)
            self.current     = abs(self.motor.current)
            self.temperature = self.motor.temperature

    def get_eased_position(self):
        if not self.move_active or self.move_start is None:
            return np.radians(self.dest_deg)
        elapsed = time.time() - self.move_start
        if elapsed >= self.movetime:
            self.move_active = False
            self.src_deg     = self.dest_deg
            return np.radians(self.dest_deg)
        s    = elapsed / self.movetime
        ease = 3*s*s - 2*s*s*s
        return np.radians(self.src_deg + ease * (self.dest_deg - self.src_deg))

    def start_move(self, from_deg, to_deg):
        self.src_deg     = from_deg
        self.dest_deg    = to_deg
        self.movetime    = max(MIN_MOVE_TIME, abs(to_deg - from_deg) * DEG_PER_SEC)
        self.move_start  = time.time()
        self.move_active = True

    def start_nudge(self):
        self.start_move(0.0, self.nudge_deg)

    def send_command(self):
        self.motor.send_mit_command(
            position=self.get_eased_position(), velocity=0.0,
            kp=min(450, KP), kd=KD, torque=0.0,
        )

    def hold_at_zero(self):
        """Hold at hold_deg (= nudge_deg after homing, or 0.0 if nudge=0)."""
        self.motor.send_mit_command(np.radians(self.hold_deg), 0.0, KP, KD, 0.0)

    def hold_at_boot_zero(self):
        self.motor.send_mit_command(0.0, 0.0, KP, KD, 0.0)

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

    def disable(self):
        try:
            self.motor.send_mit_command(0, 0, 0, 5, 0)
            time.sleep(0.1)
            self.motor.disable()
        except Exception:
            pass


# ── Bus Controller (runs in its own thread during homing) ──
class BusController:
    def __init__(self, can_interface, bus, phase1_config, phase2_config, print_lock):
        self.iface      = can_interface
        self.print_lock = print_lock
        self.units      = {}
        self.phase1_ids = [c[0] for c in phase1_config]
        self.phase2_ids = [c[0] for c in phase2_config]

        for cfg in phase1_config + phase2_config:
            mid, first, step, trig, nudge = cfg
            u = MotorUnit(bus, mid, can_interface, first, step, trig, nudge)
            self.units[mid] = u

    def _log(self, msg):
        with self.print_lock:
            print(msg)

    def init_all(self):
        for u in self.units.values():
            u.init()
            self._log(f"  [{self.iface}] M{u.motor_id} ✓ | trigger: {u.trigger_current}A | nudge: {u.nudge_deg:+.1f}°")

    # ── Core sequence runner ────────────────────────────────
    def _run_sequence(self, sequence_ids, hold_at_nudge_ids, phase_name):
        """
        sequence_ids      : motors to home one-by-one
        hold_at_nudge_ids : motors already homed — hold at their nudge_deg
        all others        : hold at boot zero
        """
        zeroed_ids  = set()
        nudging_ids = set()
        active_idx  = 0
        last_status = 0.0

        first_u = self.units[sequence_ids[0]]
        first_u.start_move(first_u.position, first_u.dest_deg)
        self._log(f"\n[{self.iface}] {phase_name} ▶ M{sequence_ids[0]} → {first_u.dest_deg:.1f}°")

        while True:
            loop_start = time.time()

            # ── Read ALL motors on this bus ─────────────────
            for u in self.units.values():
                u.read_feedback()

            # ── Command ALL motors on this bus ──────────────
            for motor_id, u in self.units.items():
                if motor_id in zeroed_ids:
                    u.hold_at_zero()               # done this phase → hold at nudge
                elif motor_id in nudging_ids:
                    u.send_command()               # nudging after zero set
                elif motor_id in hold_at_nudge_ids:
                    u.hold_at_zero()               # phase 1 done → hold at nudge during phase 2
                elif active_idx < len(sequence_ids) and motor_id == sequence_ids[active_idx]:
                    u.send_command()               # currently homing
                else:
                    u.hold_at_boot_zero()          # waiting for its turn

            # ── Nudge completion ────────────────────────────
            for motor_id in list(nudging_ids):
                u = self.units[motor_id]
                if not u.move_active:
                    u.hold_deg = u.nudge_deg
                    self._log(f"  ↩️  [{self.iface}] M{motor_id} nudge done → holding at {u.nudge_deg:+.1f}°")
                    nudging_ids.discard(motor_id)
                    zeroed_ids.add(motor_id)

            # ── Active motor logic ──────────────────────────
            if active_idx < len(sequence_ids):
                active_id = sequence_ids[active_idx]
                active_u  = self.units[active_id]
                triggered = active_u.check_current_trigger()

                # Reached dest, current still safe → chain next step
                if not active_u.move_active and not triggered \
                        and active_u.current < active_u.trigger_current \
                        and active_id not in zeroed_ids and active_id not in nudging_ids:
                    next_target = active_u.dest_deg + active_u.step_deg
                    self._log(f"  📍 [{self.iface}] M{active_id}: {active_u.dest_deg:.1f}° → {next_target:.1f}°")
                    active_u.start_move(active_u.dest_deg, next_target)

                # Current trigger → set zero → nudge → hand off
                if triggered:
                    self._log(f"  ✅ [{self.iface}] M{active_id}: {active_u.current:.2f}A >= {active_u.trigger_current}A → zero at {active_u.position:.1f}°")
                    active_u.motor.set_zero_position()
                    active_u.zeroed = True
                    active_idx += 1

                    if active_u.nudge_deg != 0.0:
                        nudging_ids.add(active_id)
                        active_u.start_nudge()
                        self._log(f"     ↩️  [{self.iface}] M{active_id} nudge: 0° → {active_u.nudge_deg:+.1f}°")
                    else:
                        active_u.hold_deg = 0.0
                        zeroed_ids.add(active_id)
                        self._log(f"     [{self.iface}] M{active_id} nudge=0° → holding at new zero")

                    if active_idx < len(sequence_ids):
                        next_id = sequence_ids[active_idx]
                        next_u  = self.units[next_id]
                        next_u.start_move(next_u.position, next_u.dest_deg)
                        self._log(f"  🚀 [{self.iface}] Next: M{next_id} → {next_u.dest_deg:.1f}°")

            # ── Periodic status ─────────────────────────────
            now = time.time()
            if now - last_status > 1.0:
                last_status = now
                parts = []
                for mid in sequence_ids:
                    u = self.units[mid]
                    if mid in zeroed_ids:
                        s = "✅"
                    elif mid in nudging_ids:
                        s = "↩️ "
                    elif u.move_active:
                        s = "🚀"
                    else:
                        s = "⏸️ "
                    parts.append(f"{s}M{mid}:{u.position:.1f}°|{u.current:.2f}A")
                self._log(f"  [{self.iface}] {phase_name}: " + "  ".join(parts))

            # ── Phase complete ──────────────────────────────
            if active_idx >= len(sequence_ids) and not nudging_ids:
                break

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, LOOP_DT - elapsed))

    # ── Thread entry: phase1 → barrier → phase2 → exit ─────
    def thread_run(self, phase1_barrier):
        self._run_sequence(
            sequence_ids      = self.phase1_ids,
            hold_at_nudge_ids = set(),              # nothing to hold at nudge yet
            phase_name        = "Phase1"
        )
        self._log(f"\n⏳ [{self.iface}] Phase 1 done — waiting at barrier for other bus...")
        phase1_barrier.wait()                       # ← HARD BARRIER
        self._log(f"\n🚦 [{self.iface}] Both buses ready — Phase 2 start!")

        self._run_sequence(
            sequence_ids      = self.phase2_ids,
            hold_at_nudge_ids = set(self.phase1_ids),  # phase1 motors hold at nudge
            phase_name        = "Phase2"
        )
        self._log(f"\n🏁 [{self.iface}] Phase 2 complete!")

    # ── Final hold (called from main thread) ───────────────
    def hold_all(self):
        for u in self.units.values():
            u.hold_at_zero()    # hold_deg is set for all motors after homing

    def disable_all(self):
        for u in self.units.values():
            u.disable()
            self._log(f"  M{u.motor_id} disabled ✓")


# ── Main ───────────────────────────────────────────────────
def main():
    print("=" * 65)
    print("🤖  AK60  |  4-LEG HOMING  |  12 MOTORS  |  2 CAN BUSES")
    print("   Phase 1: Front Right (can1: M8→M9→M7)")
    print("            Back Left   (can0: M5→M6→M4)  ← simultaneous")
    print("   Phase 2: Back Right  (can1: M11→M12→M10)")
    print("            Front Left  (can0: M2→M3→M1)   ← simultaneous")
    print("   Final:   Single thread holds all 12 at nudge positions")
    print("=" * 65)

    print_lock = threading.Lock()

    bus0 = can.interface.Bus(channel='can0', interface='socketcan', bitrate=1_000_000)
    bus1 = can.interface.Bus(channel='can1', interface='socketcan', bitrate=1_000_000)

    ctrl0 = BusController('can0', bus0,
                          CAN_CONFIG['can0']['phase1'],
                          CAN_CONFIG['can0']['phase2'],
                          print_lock)
    ctrl1 = BusController('can1', bus1,
                          CAN_CONFIG['can1']['phase1'],
                          CAN_CONFIG['can1']['phase2'],
                          print_lock)

    # ── Init ALL 12 motors ──────────────────────────────────
    print("\n🔌 Enabling all 12 motors...")
    ctrl0.init_all()
    ctrl1.init_all()
    print("✅ All 12 motors enabled — holding at boot zero\n")

    # ── Phase barrier (both buses must reach it before phase 2) ──
    phase1_barrier = threading.Barrier(2)

    # ── Start homing threads ────────────────────────────────
    t0 = threading.Thread(target=ctrl0.thread_run, args=(phase1_barrier,), daemon=True)
    t1 = threading.Thread(target=ctrl1.thread_run, args=(phase1_barrier,), daemon=True)
    t0.start()
    t1.start()
    t0.join()
    t1.join()

    # ── All homed — single thread holds forever ─────────────
    print("\n" + "=" * 65)
    print("🏁 ALL 12 MOTORS HOMED — holding forever. Ctrl+C to exit.")
    print("=" * 65)

    try:
        while True:
            ctrl0.hold_all()
            ctrl1.hold_all()
            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        print("\n🛑 Ctrl+C — releasing all motors...")

    finally:
        ctrl0.disable_all()
        ctrl1.disable_all()
        bus0.shutdown()
        bus1.shutdown()
        print("✅ Done!")


if __name__ == "__main__":
    main()
