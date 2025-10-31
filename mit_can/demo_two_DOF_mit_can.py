from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can

# CHANGE THESE TO MATCH YOUR DEVICE!
ID_1 = 4
ID_2 = 12
ID_3 = 10

Type_1 = 'AK70-10'
Type_2 = 'AK60-6'
Type_3 = 'AK60-6'




def two_DOF(dev1,dev2,dev3):
    dev1.set_zero_position()
    dev2.set_zero_position()
    dev3.set_zero_position()
    time.sleep(1.5) # wait for the motors to zero (~1 second)
    dev1.set_impedance_gains_real_unit(K=3.0,B=0.5)
    dev2.set_impedance_gains_real_unit(K=3.0,B=0.5)
    dev3.set_impedance_gains_real_unit(K=3.0,B=1.3)
    
    print("Starting 2 DOF demo. Press ctrl+C to quit.")

    loop = SoftRealtimeLoop(dt = 0.005, report=True, fade=0)
    for t in loop:
        dev1.update()
        dev2.update()
        if t < 1.0:
            dev1.position = 0.0
            dev2.position = 0.0
            dev3.position = 0.0
        elif t>=1.0 and t<=7.0:
            dev1.position = -(np.pi/2)
            dev2.position = (np.pi/3)
            dev3.position = 0.0
        else:
            dev2.position = 0
            dev1.position =  0
            dev3.position = 0.0
    del loop

if __name__ == '__main__':
    # to use additional motors, simply add another with block
    # remember to give each motor a different log name!
    with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_1) as dev1:
        with TMotorManager_mit_can(motor_type=Type_2, motor_ID=ID_2) as dev2:
            with TMotorManager_mit_can(motor_type=Type_3, motor_ID=ID_3) as dev3:
               two_DOF(dev1,dev2,dev3)
