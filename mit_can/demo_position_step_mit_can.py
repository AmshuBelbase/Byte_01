from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can
import warnings


# CHANGE THESE TO MATCH YOUR DEVICE!
Type = 'AK70-10'
ID = 2


def position_step(dev):
    dev.set_zero_position() # has a delay!
    time.sleep(5)
    dev.set_impedance_gains_real_unit(K=40.0,B=4.0)
    
    print("Starting position step demo. Press ctrl+C to quit.")

    loop = SoftRealtimeLoop(dt = 0.02, report=True, fade=0)
    for t in loop:
    	try:
    		with warnings.catch_warnings():
    			warnings.simplefilter("ignore", RuntimeWarning)
    			dev.update()
    			if t < 1.0:
    				dev.position = 0.0
    			elif t < 3.0:
    				dev.position = np.radians(30)
    			elif t >3.0 and t<6.0:
    				dev.position = np.radians(120)
    			else:
    				dev.position = np.radians(360) 
    
    	except RuntimeWarning as rw:
    		print("Warn: ", rw)
    		continue
    	except Exception as e:
    		print("Error: ", e)
    		continue
    	
    	time.sleep(0.01)
        

            

    del loop

if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev:
        position_step(dev)
