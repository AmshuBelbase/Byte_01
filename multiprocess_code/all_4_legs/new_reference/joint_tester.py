#!/usr/bin/env python3

import sys
import json
import multiprocessing as mp
import pickle
import queue
import socket
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from can_runtime import BusRuntime
from homing_controller import BusHomingController, ControlTuning
from robot_config import SOCKET_HOST, SOCKET_PORT, SIT_TARGETS_DEG
from main_with_ik import (
    CAN_CONFIG, 
    parse_calibration_flag, 
    collect_motor_ids,
    controller_thread_entry,
    load_motor_command_config,
    runtime_for_motor_id,
    send_live_targets,
    drain_latest_packet,
    limit_target_step,
    initialize_live_command_state
)

LIVE_QUEUE_MAXSIZE = 1
MOTOR_CONFIG_PATH = "motor_config.json"
MAX_LIVE_DEG_PER_S = 30.0  # Kept slow for safe testing

def validate_joint_payload(payload: Any) -> Tuple[Dict[int, float], Optional[float]]:
    """Validates payload dict of {motor_id: target_deg}"""
    if not isinstance(payload, dict):
        raise ValueError("payload must be a dict")

    commands: Dict[int, float] = {}
    for key, val in payload.items():
        if key == "speed": continue
        try:
            mid = int(key)
            if 1 <= mid <= 12:
                commands[mid] = float(val)
        except ValueError:
            pass

    speed_override = payload.get("speed", None)
    if speed_override is not None:
        speed_override = float(speed_override)

    return commands, speed_override

def socket_listener_process(dest_queue: mp.Queue, stop_event: mp.Event, port: int, host: str):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.settimeout(0.2)
    try:
        server.bind((host, port))
        server.listen(5)
        print(f"[Joint Socket] Listening on {host}:{port} for direct angles", flush=True)

        while not stop_event.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            with conn:
                conn.settimeout(0.2)
                data = bytearray()
                while not stop_event.is_set():
                    try:
                        chunk = conn.recv(4096)
                        if not chunk: break
                        data.extend(chunk)
                    except socket.timeout:
                        continue
            if not data: continue

            try:
                payload = pickle.loads(bytes(data))
                commands, speed = validate_joint_payload(payload)
                
                # Drain queue and put fresh command
                while True:
                    try: dest_queue.get_nowait()
                    except queue.Empty: break
                dest_queue.put_nowait((commands, speed))
            except Exception as exc:
                print(f"[Joint Socket] Drop invalid payload: {exc}", flush=True)
    finally:
        server.close()

def run_joint_live_control(
    ctrl0, ctrl1, rt0, rt1, tuning, stop_event, live_queue, motor_config
):
    live_cmd_deg = initialize_live_command_state(rt0, rt1)
    
    print("\n[PHASE 1] Holding homed positions for 1s...")
    hold_start = time.time()
    while (time.time() - hold_start) < 1.0 and not stop_event.is_set():
        ctrl0.hold_all_once()
        ctrl1.hold_all_once()
        time.sleep(tuning.loop_dt)

    print("\n[PHASE 2] Moving to default sitting posture...")
    arrived_at_sit = False
    while not arrived_at_sit and not stop_event.is_set():
        arrived_at_sit = True
        for motor_id in range(1, 13):
            live_cmd_deg[motor_id] = limit_target_step(
                current_cmd_deg=live_cmd_deg[motor_id],
                requested_deg=SIT_TARGETS_DEG[motor_id],
                max_deg_per_s=MAX_LIVE_DEG_PER_S * motor_config[motor_id]["gear_ratio"],
                dt=tuning.loop_dt,
            )
            if abs(live_cmd_deg[motor_id] - SIT_TARGETS_DEG[motor_id]) > 0.001:
                arrived_at_sit = False
        send_live_targets(rt0, rt1, live_cmd_deg, motor_config)
        time.sleep(tuning.loop_dt)

    time.sleep(0.5)
    print("\n[PHASE 3] Zeroing all motors (New abstract 0.0)...")
    for motor_id in range(1, 13):
        runtime = runtime_for_motor_id(motor_id, rt0, rt1)
        runtime.zero_motor(motor_id, permanent=False)
        live_cmd_deg[motor_id] = 0.0

    time.sleep(1.0)
    print("\n✅ Ready for direct joint testing. Send commands via joint_sender.py")

    # This stores the MATHEMATICAL theta target (0.0 for all by default)
    abstract_targets = {i: 0.0 for i in range(1, 13)}
    current_speed_limit = MAX_LIVE_DEG_PER_S

    while not stop_event.is_set():
        item = drain_latest_packet(live_queue)
        if item is not None:
            commands, speed_override = item
            for mid, ang in commands.items():
                abstract_targets[mid] = ang
            current_speed_limit = speed_override if speed_override is not None else MAX_LIVE_DEG_PER_S
            print(f"🎯 New Abstract Targets applied: {commands}")

        for motor_id in range(1, 13):
            cfg = motor_config[motor_id]
            raw_target = abstract_targets[motor_id]
            
            # Translate abstract theta into raw motor encoder target
            if bool(cfg["flipped"]):
                raw_target = -raw_target
            raw_target *= float(cfg["gear_ratio"])

            live_cmd_deg[motor_id] = limit_target_step(
                current_cmd_deg=live_cmd_deg[motor_id],
                requested_deg=raw_target,
                max_deg_per_s=current_speed_limit * cfg["gear_ratio"],
                dt=tuning.loop_dt,
            )

        send_live_targets(rt0, rt1, live_cmd_deg, motor_config)
        time.sleep(tuning.loop_dt)

def main():
    CALIBRATION_REQUIRED = parse_calibration_flag()
    print("=== DIRECT JOINT TESTER ===")
    
    stop_event = threading.Event()
    errors: List[str] = []
    tuning = ControlTuning(
        active_kp=114.0, active_kd=1.2, hold_kp=54.0, hold_kd=0.9, homed_hold_kp=84.0, homed_hold_kd=1.2, loop_hz=100.0, min_move_time_s=1.2, seconds_per_deg=0.03, trigger_confirm_s=0.15, trigger_velocity_raw_max=4.0
    )

    rt0 = rt1 = live_socket_process = None
    try:
        rt0 = BusRuntime("can0", collect_motor_ids(CAN_CONFIG["can0"]), bitrate=1_000_000)
        rt1 = BusRuntime("can1", collect_motor_ids(CAN_CONFIG["can1"]), bitrate=1_000_000)

        ctrl0 = BusHomingController(rt0, CAN_CONFIG["can0"]["phase1"], CAN_CONFIG["can0"]["phase2"], tuning)
        ctrl1 = BusHomingController(rt1, CAN_CONFIG["can1"]["phase1"], CAN_CONFIG["can1"]["phase2"], tuning)

        if CALIBRATION_REQUIRED:
            print("\n🚀 Homing...")
            # Blocking homing for simplicity in tester script
            ctrl0.run_all_phases()
            ctrl1.run_all_phases()
            for _ in range(5):
                ctrl0.hold_all_once()
                ctrl1.hold_all_once()
                time.sleep(tuning.loop_dt)

        motor_config = load_motor_command_config(MOTOR_CONFIG_PATH)

        live_queue = mp.Queue(maxsize=1)
        live_socket_stop = mp.Event()
        live_socket_process = mp.Process(
            target=socket_listener_process,
            args=(live_queue, live_socket_stop, SOCKET_PORT, SOCKET_HOST),
            daemon=True,
        )
        live_socket_process.start()

        run_joint_live_control(ctrl0, ctrl1, rt0, rt1, tuning, stop_event, live_queue, motor_config)

    except KeyboardInterrupt:
        stop_event.set()
    finally:
        if live_socket_process: live_socket_process.terminate()
        if rt0: rt0.disable_all(); rt0.stop()
        if rt1: rt1.disable_all(); rt1.stop()
        print("✅ Shutdown complete.")

if __name__ == "__main__":
    main()