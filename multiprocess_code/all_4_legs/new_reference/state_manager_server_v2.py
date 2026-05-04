import pickle
import socket
import asyncio
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from config.robot_config import SOCKET_HOST, SOCKET_PORT, LEG_ORDER, MAX_LIVE_DEG_PER_S
from lib.trot_gait import TrotGaitController, RotationGaitController
import time
import copy
from mpu_leveling import MPULeveler


# --- Data Models ---


class StateRequest(BaseModel):
    new_state: str
    sender_id: str
    cycles: int


# --- Core Logic ---


class QuadrupedStateManager:
    def __init__(self):
        # Internal tracking of the current position
        self.current_positions = {leg: [0.0, 0.0, 0.0] for leg in LEG_ORDER}

        # Define Coordinate targets
        self.static_states = {
            "SIT":   {"dx": 4.0, "dy": 0.0, "dz": 0.0},
            "STAND": {"dx": 4.0, "dy": 0.0, "dz": -14.0},
        }

        self.dynamic_states = {
            "FORWARD":   {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "RIGHT":     {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "BACKWARD":  {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "LEFT":      {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "JUMP":      {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "CW": {"dx": 0.0, "dy": 0.0, "dz": 0.0},   # NEW
            "CCW":{"dx": 0.0, "dy": 0.0, "dz": 0.0},   # NEW
        }

        # The Lock ensures only one movement happens at a time
        self.lock = asyncio.Lock()

        # MPU leveler — reference is captured on SIT, correction applied on STAND
        self.leveler = MPULeveler()


    # ─────────────────────────────────────────────────────────────────────────────
    # CORE TROT GAIT RUNNER  (translation — forward / backward / strafe)
    # ─────────────────────────────────────────────────────────────────────────────
    async def _run_trot_gait_cycle(self, axis: str, direction: int):
        """
        Always completes a full cycle before stopping — all feet land cleanly.
        axis      : 'x' fore-aft  |  'y' lateral (strafe)
        direction : +1 forward/left  |  -1 backward/right
        """
        GAIT_SPEED_DEG_PER_S = 500.0
        cycle_time = 0.4
        dt = 0.01
        no_of_updates = int(cycle_time / dt)
        phase_step = 1.0 / no_of_updates
        phase = 0.0

        # Create a controller instance for each leg
        controller = {}
        for leg in LEG_ORDER:
            controller[leg] = TrotGaitController(leg, axis, direction)

        while phase < 1.0:
            payload_deltas = {}

            for leg in LEG_ORDER:
                dx, dy, dz = controller[leg]._foot_pos_phase(phase)

                self.current_positions[leg][0] += dx
                self.current_positions[leg][1] += dy
                self.current_positions[leg][2] += dz

                payload_deltas[leg] = self.current_positions[leg]

            success = await self._send_to_robot_async(payload_deltas, speed=GAIT_SPEED_DEG_PER_S)
            print("\n")
            await asyncio.sleep(dt)
            phase += phase_step

        # Return all feet to STAND after the gait cycle
        target = self.static_states["STAND"]
        payload_deltas = {}
        for leg in LEG_ORDER:
            payload_deltas[leg] = [target["dx"], target["dy"], target["dz"]]
            self.current_positions[leg] = payload_deltas[leg]

        success = await self._send_to_robot_async(payload_deltas)
        phase = 0.0


    # ─────────────────────────────────────────────────────────────────────────────
    # ROTATION GAIT RUNNER  (NEW — in-place CW / CCW yaw)
    # ─────────────────────────────────────────────────────────────────────────────
    async def _run_rotation_gait_cycle(self, direction: int):
        """
        Runs one full in-place rotation cycle using RotationGaitController.
        direction : +1 → CW (viewed from above)  |  -1 → CCW
        """
        GAIT_SPEED_DEG_PER_S = 500.0
        cycle_time = 0.4
        dt = 0.01
        no_of_updates = int(cycle_time / dt)
        phase_step = 1.0 / no_of_updates
        phase = 0.0

        # Create a rotation controller instance for each leg
        controller = {}
        for leg in LEG_ORDER:
            controller[leg] = RotationGaitController(leg, direction)

        while phase < 1.0:
            payload_deltas = {}

            for leg in LEG_ORDER:
                dx, dy, dz = controller[leg]._foot_pos_phase(phase)

                self.current_positions[leg][0] += dx
                self.current_positions[leg][1] += dy
                self.current_positions[leg][2] += dz

                payload_deltas[leg] = self.current_positions[leg]

            success = await self._send_to_robot_async(payload_deltas, speed=GAIT_SPEED_DEG_PER_S)
            print("\n")
            await asyncio.sleep(dt)
            phase += phase_step

        # Return all feet to STAND after the rotation cycle
        target = self.static_states["STAND"]
        payload_deltas = {}
        for leg in LEG_ORDER:
            payload_deltas[leg] = [target["dx"], target["dy"], target["dz"]]
            self.current_positions[leg] = payload_deltas[leg]

        success = await self._send_to_robot_async(payload_deltas)
        phase = 0.0


    # ─────────────────────────────────────────────────────────────────────────────
    # STATE MACHINE
    # ─────────────────────────────────────────────────────────────────────────────
    async def change_state(self, target_state: str, sender: str, cycles: int = 1):
        """API-like function to trigger a state transition with concurrency protection."""

        if self.lock.locked():
            raise HTTPException(
                status_code=429,
                detail=f"Robot is busy. Request from {sender} rejected."
            )

        target_state = target_state.upper()
        async with self.lock:
            if target_state not in self.static_states and target_state not in self.dynamic_states:
                raise HTTPException(status_code=400, detail=f"State '{target_state}' unknown.")

            if target_state in self.static_states:
                target = self.static_states[target_state]
                payload_deltas = {}

                for leg in LEG_ORDER:
                    payload_deltas[leg] = [target["dx"], target["dy"], target["dz"]]
                    self.current_positions[leg] = payload_deltas[leg]

                success = await self._send_to_robot_async(payload_deltas)

                if not success:
                    raise HTTPException(status_code=503, detail="Robot hardware communication failed.")

                await asyncio.sleep(2.0)

                if target_state == "SIT":
                    print("[State] Bot is sitting — capturing MPU reference plane...")
                    await asyncio.get_event_loop().run_in_executor(
                        None, self.leveler.capture_reference
                    )
                    print("[State] MPU reference captured.")

                elif target_state == "STAND":
                    print("[State] Bot is standing — applying MPU level correction...")
                    await asyncio.get_event_loop().run_in_executor(
                        None, self.leveler.level_after_stand
                    )
                    print("[State] MPU level correction complete.")

                return {"status": "success", "reached": target_state}

            else:
                # Dynamic states — run the requested gait for the given number of cycles
                while cycles > 0:
                    if target_state == "FORWARD":
                        await self._run_trot_gait_cycle(axis='x', direction=1)
                    elif target_state == "BACKWARD":
                        await self._run_trot_gait_cycle(axis='x', direction=-1)
                    elif target_state == "RIGHT":
                        await self._run_trot_gait_cycle(axis='y', direction=1)
                    elif target_state == "LEFT":
                        await self._run_trot_gait_cycle(axis='y', direction=-1)
                    elif target_state == "CW":        # NEW
                        await self._run_rotation_gait_cycle(direction=+1)
                    elif target_state == "CCW":       # NEW
                        await self._run_rotation_gait_cycle(direction=-1)
                    else:
                        return {"status": "error", "detail": f"Dynamic state '{target_state}' not implemented."}
                    cycles -= 1

                return {"status": "success", "started": target_state}


    async def _send_to_robot_async(self, payload: dict, speed: float = MAX_LIVE_DEG_PER_S):
        try:
            payload["speed"] = speed
            data = pickle.dumps(payload)
            reader, writer = await asyncio.wait_for(
                asyncio.open_connection(SOCKET_HOST, SOCKET_PORT),
                timeout=1.0
            )
            writer.write(data)
            await writer.drain()
            writer.close()
            await writer.wait_closed()
            return True
        except Exception as e:
            print(f"Socket Error: {e}")
            return False


# --- FastAPI App ---


app = FastAPI(title="Quadruped API")
robot_manager = QuadrupedStateManager()


@app.post("/change-state")
async def handle_state_change(request: StateRequest):
    result = await robot_manager.change_state(request.new_state, request.sender_id, request.cycles)
    print(f"Received request from {request.sender_id} to change state to '{request.new_state}' for {request.cycles} cycles.")
    return {
        "sender_id": request.sender_id,
        "requested_state": request.new_state,
        "cycles": request.cycles,
        "result": "received"
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)