from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can

# CHANGE THESE TO MATCH YOUR DEVICES!
Type1 = 'AK60-6'
ID1 = 11

Type2 = 'AK60-6'
ID2 = 12   # <-- change this to your second motor's ID


def position_step(dev1, dev2):
    # Zero both motors
    dev1.set_zero_position()
    dev2.set_zero_position()
    time.sleep(1.5)

    # Set impedance gains for both
    dev1.set_impedance_gains_real_unit(K=2, B=0.5)
    dev2.set_impedance_gains_real_unit(K=2, B=0.5)
    
    print("Motor1 moves first, then Motor2 after 2s delay. Ctrl+C to quit.")

    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)
    for t in loop:
        dev1.update()
        dev2.update()

        if t < 1.0:
            # Initial: both stopped at 0
            dev1.position = 0.0
            dev2.position = 0.0

        elif t < 3.0:
            # Motor1 rotates to 180°, Motor2 stays at 0
            dev1.position = np.pi        # 180 degrees
            dev2.position = 0.0

        else:
            # Motor2 rotates to 180°, Motor1 holds its last position
            dev1.position = np.pi        # stays at 180
            dev2.position = np.pi        # goes to 180

    del loop


if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type1, motor_ID=ID1) as dev1, \
         TMotorManager_mit_can(motor_type=Type2, motor_ID=ID2) as dev2:
        position_step(dev1, dev2)
