#!/usr/bin/env python3

import threading
import time
from typing import Dict, List

from can_runtime import BusRuntime
from homing_controller import BusHomingController, ControlTuning, HomingMotorConfig


CAN_CONFIG: Dict[str, Dict[str, List[HomingMotorConfig]]] = {
    'can0': {
        'phase1': [
            HomingMotorConfig(5, -60.0, -5.0, 3.0,  70.0),
            HomingMotorConfig(6,  10.0,  5.0, 5.0, -38.0),
            HomingMotorConfig(4,  60.0,  5.0, 4.0, -60.0),
        ],
        'phase2': [
            HomingMotorConfig(2, -60.0, -5.0, 3.0,  70.0),
            HomingMotorConfig(3,  10.0,  5.0, 5.0, -38.0),
            HomingMotorConfig(1,  60.0,  5.0, 4.0, -60.0),
        ],
    },
    'can1': {
        'phase1': [
            HomingMotorConfig(8,   60.0,  5.0, 3.0, -70.0),
            HomingMotorConfig(9,  -10.0, -5.0, 5.0,  38.0),
            HomingMotorConfig(7,  -60.0, -5.0, 4.0,  60.0),
        ],
        'phase2': [
            HomingMotorConfig(11,  60.0,  5.0, 3.0, -70.0),
            HomingMotorConfig(12, -10.0, -5.0, 5.0,  38.0),
            HomingMotorConfig(10, -60.0, -5.0, 4.0,  60.0),
        ],
    },
}


def collect_motor_ids(bus_cfg: Dict[str, List[HomingMotorConfig]]) -> List[int]:
    ids = []
    for phase_name in ('phase1', 'phase2'):
        ids.extend(cfg.motor_id for cfg in bus_cfg[phase_name])
    return sorted(set(ids))


def controller_thread_entry(controller, barrier, stop_event, errors):
    try:
        controller.thread_run(barrier, stop_event)
    except Exception as exc:
        errors.append(f"{controller.runtime.channel}: {exc}")
        stop_event.set()


def main():
    print("=" * 70)
    print("🤖 AK60 | 4-LEG HOMING | 12 MOTORS | 2 CAN BUSES")
    print("   Phase 1: Front Right (can1: M8→M9→M7)")
    print("            Back Left   (can0: M5→M6→M4)  ← simultaneous")
    print("   Phase 2: Back Right  (can1: M11→M12→M10)")
    print("            Front Left  (can0: M2→M3→M1)  ← simultaneous")
    print("   Final:    Single thread holds all 12 at nudge positions")
    print("=" * 70)

    print_lock = threading.Lock()
    stop_event = threading.Event()
    errors = []

    tuning = ControlTuning(
        kp=200.0,
        kd=1.5,
        loop_hz=100.0,
        min_move_time_s=1.0,
        seconds_per_deg=0.02,
        trigger_confirm_s=0.30,
        feedback_timeout_s=0.12,
        bus_silence_timeout_s=0.30,
        startup_timeout_s=3.0,
        startup_poll_s=0.01,
        zero_feedback_timeout_s=0.50,
        max_search_time_s=20.0,
        max_search_travel_deg=220.0,
        status_period_s=1.0,
        safe_idle_kp=40.0,
        safe_idle_kd=1.0,
    )

    rt0 = None
    rt1 = None

    try:
        can0_ids = collect_motor_ids(CAN_CONFIG['can0'])
        can1_ids = collect_motor_ids(CAN_CONFIG['can1'])

        rt0 = BusRuntime('can0', can0_ids, bitrate=1_000_000)
        rt1 = BusRuntime('can1', can1_ids, bitrate=1_000_000)

        ctrl0 = BusHomingController(
            runtime=rt0,
            phase1_config=CAN_CONFIG['can0']['phase1'],
            phase2_config=CAN_CONFIG['can0']['phase2'],
            tuning=tuning,
            print_lock=print_lock,
        )
        ctrl1 = BusHomingController(
            runtime=rt1,
            phase1_config=CAN_CONFIG['can1']['phase1'],
            phase2_config=CAN_CONFIG['can1']['phase2'],
            tuning=tuning,
            print_lock=print_lock,
        )

        print("\n📡 Waiting for periodic feedback from all motors...")
        ctrl0.verify_periodic_feedback_and_capture_boot_holds()
        ctrl1.verify_periodic_feedback_and_capture_boot_holds()
        print("✅ All motors are streaming feedback.\n")

        print("🧷 Sending low-gain idle hold at captured boot positions...")
        for _ in range(20):
            ctrl0.send_idle_hold_once()
            ctrl1.send_idle_hold_once()
            time.sleep(tuning.loop_dt)

        phase1_barrier = threading.Barrier(2)

        t0 = threading.Thread(
            target=controller_thread_entry,
            args=(ctrl0, phase1_barrier, stop_event, errors),
            daemon=True
        )
        t1 = threading.Thread(
            target=controller_thread_entry,
            args=(ctrl1, phase1_barrier, stop_event, errors),
            daemon=True
        )

        t0.start()
        t1.start()
        t0.join()
        t1.join()

        if errors:
            raise RuntimeError(" | ".join(errors))

        print("\n" + "=" * 70)
        print("🏁 ALL 12 MOTORS HOMED — holding forever. Ctrl+C to exit.")
        print("=" * 70)

        while not stop_event.is_set():
            ctrl0.hold_all_once()
            ctrl1.hold_all_once()
            time.sleep(tuning.loop_dt)

    except KeyboardInterrupt:
        print("\n🛑 Ctrl+C — stopping...")
        stop_event.set()

    except Exception as exc:
        print(f"\n❌ Runtime error: {exc}")
        stop_event.set()

    finally:
        print("\nReleasing motors and shutting down...")
        try:
            if rt0 is not None:
                rt0.disable_all()
        except Exception:
            pass

        try:
            if rt1 is not None:
                rt1.disable_all()
        except Exception:
            pass

        try:
            if rt0 is not None:
                rt0.stop()
        except Exception:
            pass

        try:
            if rt1 is not None:
                rt1.stop()
        except Exception:
            pass

        print("✅ Done.")


if __name__ == "__main__":
    main()
