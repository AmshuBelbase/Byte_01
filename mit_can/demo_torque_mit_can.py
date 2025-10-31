from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can

# CHANGE THESE TO MATCH YOUR DEVICE!
Type1 = 'AK70-10'
ID1 = 2

#Type2 = 'AK60-6'
#ID2 = 11


def torque_step(dev):
    dev.set_zero_position()
    time.sleep(1.5) # wait for the motor to zero (~1 second)
    dev.set_current_gains()
    dev.set_impedance_gains_real_unit(K=6.0, B=2.0)
    
    
    print("Starting torque step demo. Press ctrl+C to quit.")
    loop = SoftRealtimeLoop(dt = 0.01, report=True, fade=0)
    for t in loop:
        dev.update()
        if t < 1.0:
            dev.torque = 0.0
            dev.position = 0.0
        else:
            dev.torque = -1.0
            dev.position = np.radians(30)

    del loop


if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type1, motor_ID=ID1) as dev1:
        torque_step(dev1)
    #with TMotorManager_mit_can(motor_type=Type2, motor_ID=ID2) as dev2:
        #torque_step(dev2)
