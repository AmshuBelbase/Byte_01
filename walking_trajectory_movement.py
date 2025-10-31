from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can
import _3dof_ik_with_shift_circle_method as ik

# CHANGE THESE TO MATCH YOUR DEVICES!
Type1 = 'AK70-10'
ID1 = 2

Type2 = 'AK60-6'
ID2 = 5

Type3 = 'AK60-6'
ID3 = 11

# Link lengths in cm
L1 = 5.995  # Link 1 length
linkConst = 9.094  # Constant link between L1 and L2 (RADIUS OF CIRCLE WHEN L1 IS ROTATED ALONG Z AXIS)
L2 = 22  # Link 2 length
L3 = 21.5  # Link 3 length

# Trajectory parameters
x = 9.094
z_start, z_end = linkConst, 0 # right to left co-ordinates of bots leg for movement
y_start, y_end = 28, 23 # max up and max low of bots leg
y_amplitude = y_start - y_end  # max amplitude of sine wave for movement
num_points = 50  # For smoothness

# Prepare trajectory points
zs = np.linspace(z_start, z_end, num_points) # Interpolate z from start to end (right to left)
ts = np.linspace(0, np.pi, num_points) # Generate parameter t from 0 to π (for half a sine wave)
ys = y_amplitude * np.sin(ts) # y follows a sine curve with amplitude from bottom to top to bottom
coords = [(x, y_end-y, z) for y, z in zip(ys, zs)] # Build the trajectory points

def run_trajectory(dev1, dev2, dev3):
    # Zero all motors
    dev1.set_zero_position()
    dev2.set_zero_position()
    dev3.set_zero_position()
    time.sleep(1.5)

    # Set impedance gains for all
    dev1.set_impedance_gains_real_unit(K=6.0, B=2.0)
    dev2.set_impedance_gains_real_unit(K=6.0, B=2.0)
    dev3.set_impedance_gains_real_unit(K=6.0, B=2.0)  # Adjust as needed
    
    print("Starting trajectory demo for three motors. Press ctrl+C to quit.")

    trajectory_update_interval = 0.1  # seconds, e.g., update trajectory every 0.1s (10 Hz)
    iterations_per_update = int(trajectory_update_interval / 0.01)  # 0.1/0.01 = 10

    coord_idx = 0
    update_counter = 0

    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)
    got_to_home = True

    try:
        for t in loop:
            dev1.update()
            dev2.update()
            dev3.update()

            # During the first second, hold all motors at zero position
            if t < 1.0:
                dev1.position = 0.0
                dev2.position = 0.0
                dev3.position = 0.0
                got_to_home = True
            elif got_to_home: 
                time.sleep(1.0)  # Hold for a second
                x, y, z = coords[0] # Move to first trajectory point directly
                coord_idx = 1
                theta1, theta2, theta3 = ik.inverse_kinematics(x, y, z, L1, linkConst, L2, L3)
                dev1.position = -theta1
                dev2.position = -theta2
                dev3.position = -theta3
                time.sleep(1.0)  # Hold for a second
                got_to_home = False
            else:
                # Update trajectory point only every 'iterations_per_update'
                if update_counter % iterations_per_update == 0:
                    x, y, z = coords[coord_idx]
                    theta1, theta2, theta3 = ik.inverse_kinematics(x, y, z, L1, linkConst, L2, L3)
                    coord_idx += 1
                    if coord_idx >= len(coords):
                        coord_idx = 0
                        got_to_home = True
                    else:
                        got_to_home = False

                dev1.position = -theta1
                dev2.position = -theta2
                dev3.position = -theta3
                
                update_counter += 1

    except KeyboardInterrupt:
        print("Demo stopped by user.")

    del loop


if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type1, motor_ID=ID1) as dev1, \
         TMotorManager_mit_can(motor_type=Type2, motor_ID=ID2) as dev2, \
         TMotorManager_mit_can(motor_type=Type3, motor_ID=ID3) as dev3:
        run_trajectory(dev1, dev2, dev3)

