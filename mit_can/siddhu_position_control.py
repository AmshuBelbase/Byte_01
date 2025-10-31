from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can

# CHANGE THESE TO MATCH YOUR DEVICES!
Type1 = 'AK70-10'
ID1 = 2

Type2 = 'AK60-6'
ID2 = 5   # <-- change this to your second motor's ID


def position_step(dev1, dev2):
    # Zero both motors
    dev1.set_zero_position()
    dev2.set_zero_position()
    time.sleep(1.5)

    # Set impedance gains for both
    dev1.set_impedance_gains_real_unit(K=6.0, B=2.0)
    dev2.set_impedance_gains_real_unit(K=6.0, B=2.0)
    
    print("Starting position step demo for two motors. Press ctrl+C to quit.")

    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)
    for t in loop:
        dev1.update()
        dev2.update()

        if t < 1.0:
            dev1.position = 0.0
            dev2.position = 0.0
        else:
            # <<< CHANGE TARGET POSITIONS HERE >>>
            dev1.position = np.radians(29)  # Motor 1 goes to 90 degrees
            dev2.position = np.radians(-25)  # Motor 2 goes to -90 degrees

    del loop


if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type1, motor_ID=ID1) as dev1, \
         TMotorManager_mit_can(motor_type=Type2, motor_ID=ID2) as dev2:
        position_step(dev1, dev2)

