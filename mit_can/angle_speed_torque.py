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
    
    target_angle = 90
    speed_constant = 100/360
    
    angles = np.linspace(0, target_angle, target_angle*speed_constant)
    interval_between_angles = 0.006 # seconds
    last_update = 0
    angle_idx = 0
    err_threshhold = 1
    cycle = 0
    
    for t in loop:
    	try:
    		with warnings.catch_warnings():
    			warnings.simplefilter("ignore", RuntimeWarning)
    			dev.update()
    			if t < 1.0:
    				dev.position = 0.0
    				last_update = t
    			else:
    				if t > last_update+interval_between_angles:
    					angle = 0
    					if cycle %2 == 0:
    						angle = angles[angle_idx]
    					else:
    						angle = 360 - angles[angle_idx]
    					dev.position = np.radians(angle)
    					err = np.degrees(dev.position) - angle
    					print("Time: ", t," | To be: ", angle, " | Is at: ", np.degrees(dev.position), " | Error: ", err)
    					last_update = t
    					angle_idx += 1
    					if angle_idx >= len(angles):
    						if abs(err) > err_threshhold:
    							angle_idx -= 1
    						else:
    							angle_idx = 0
    							cycle+=1
    							print(angle_idx, cycle, " ------------------- ")
    				
    				
    
    	except RuntimeWarning as rw:
    		print("Warn: ", rw)
    		continue
    	except Exception as e:
    		print("Error: ", e)
    		continue
    	
    	#time.sleep(0.01)
        

            

    del loop

if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev:
        position_step(dev)
