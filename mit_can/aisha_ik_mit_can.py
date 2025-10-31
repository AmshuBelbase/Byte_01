import time
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from TMotorCANControl.mit_can import TMotorManager_mit_can

MOTOR_TYPES = ['AK70-10', 'AK60-6', 'AK70-10', 'AK60-6']
MOTOR_IDS   = [4, 12,3,10]  

def connect_motors():
    motors = []
    for m_type, m_id in zip(MOTOR_TYPES, MOTOR_IDS):
        dev = TMotorManager_mit_can(motor_type=m_type, motor_ID=m_id)
        dev.__enter__()
        dev.set_zero_position()
        time.sleep(1.5)
        dev.set_impedance_gains_real_unit(K=3, B=1.0)
        motors.append(dev)
    return motors

def disconnect_motors(motors):
    for dev in motors:
        dev.__exit__(None, None, None)

# --- Hold unused motors at zero ---
def hold_unused_motors(all_motors, used_indices):
    for i, motor in enumerate(all_motors):
        if i not in used_indices:
            motor.update()
            motor.position = 0.0

# --- Time-based step motion for Motor 1 & 2 ---
def move_motors_step(motors, start_angles, target_angles):
    """
    motors: list of all motor objects
    start_angles: [motor1_start, motor2_start]
    target_angles: [motor1_target, motor2_target]
    """
    start_time = time.time()
    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)
    try:
        for _ in loop:
            t = time.time() - start_time

            # --- Motor 1 ---
            motors[0].update()
            if t < 1.0:
                motors[0].position = start_angles[0]
            elif t <= 6.0:
                motors[0].position = target_angles[0]
            elif t<7.0:
                motors[0].position = start_angles[0]

            # --- Motor 2 ---
            motors[1].update()
            if t < 1.0:
                motors[1].position = start_angles[1]
            elif t >= 1.0 and t<=4.0 :
                motors[1].position = target_angles[1]
            elif t>4.0 and t<=5.0 :
                motors[1].position = temp_angle
            elif t>5.0 and t<=6.0 :
                motors[1].position = target_angles[1]
            elif t<7.0:
                motor[1].position= start_angles[1]
                

            # --- Hold other motors ---
            hold_unused_motors(motors, used_indices=[0, 1])

    except KeyboardInterrupt:
        print("Motion interrupted by user.")
    del loop

# --- Run Program ---
if __name__ == "__main__":
    motors = connect_motors()
    try:
        # Define angles in radians: motor1 negative, motor2 positive
        start_angles  = [0.0, 0.0]       # home/start positions
        target_angles = [-0.72, 3.5]      # target positions
        temp_angle = 1.5

        move_motors_step(motors, start_angles, target_angles)
    finally:
        disconnect_motors(motors)
        print("Motors disconnected.")
