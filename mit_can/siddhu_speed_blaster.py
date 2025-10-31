from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can

# CHANGE THESE TO MATCH YOUR DEVICES!
Type1 = 'AK60-6'
ID1 = 11

Type2 = 'AK60-6'
ID2 = 12   # <-- change this to your second motor's ID


def speed_step(dev1, dev2):
    # Zero both motors
    dev1.set_zero_position()
    dev2.set_zero_position()
    time.sleep(1.5)  # wait for the motors to zero

    # Set speed gains
    dev1.set_speed_gains(kd=0.5)
    dev2.set_speed_gains(kd=0.5)
    
    print("Starting speed step demo for two motors. Press ctrl+C to quit.")
    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)
    for t in loop:
        dev1.update()
        dev2.update()

        if t < 1.0:
            dev1.velocity = 0.0
            dev2.velocity = 0.0
        else:
            # <<< CHANGE SPEEDS HERE >>>
            dev1.velocity = 1.0   # Motor 1 speed (in revolutions per second or rad/s depending on lib)
            dev2.velocity = -10.0  # Motor 2 speed (set your desired value)

    del loop


if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type1, motor_ID=ID1) as dev1, \
         TMotorManager_mit_can(motor_type=Type2, motor_ID=ID2) as dev2:
        speed_step(dev1, dev2)
