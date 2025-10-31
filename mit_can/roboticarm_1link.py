from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can

# Only using motor 2 for now
Type2 = 'AK60-6'
ID2 = 12  # Change to match your motor's ID

def position_step(dev2):
    # Zero motor
    dev2.set_zero_position()
    time.sleep(1.0)

    # Set impedance gains
    dev2.set_impedance_gains_real_unit(K=4.7, B=1.0)

    print("Starting position step demo. Press ctrl+C to quit.")

    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)
    for t in loop:
        dev2.update()

        if t < 1.0:
            dev2.position = 0.0
        else:
            dev2.position = np.radians(-90)  # Move to 30 degrees

    del loop


if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type2, motor_ID=ID2) as dev2:
        position_step(dev2)

