import numpy as np 
import math

# Link lengths in cm
L1 = 11  # Link 1 length
L2 = 36  # Link 2 length

# IK function: calculates joint angles theta1 (base twist) and theta2 (pitch)
def inverse_kinematics(x, y, z, L1, L2):
    theta1 = np.arctan2(y, x)  # Base twist angle
    r = np.sqrt(x**2 + y**2)   # Distance from base axis horizontally
    theta2 = np.arctan2(z-L1, r)  # Pitch angle for Link 2 rotation
    reach = np.sqrt((z - L1)**2 + r**2)

    if reach > L2 + 1e-9:
        raise ValueError("Target out of reach for link L2")
    return theta1, theta2 


x0, y0, z0 = 47, 0, 0

# Example target position (x,y,z) in cm

# x1, y1, z1 = 47, 0, 0 # (0,0) 

# z1, y1, x1 = 25.7, 28.2, 16.3 # (65,30) 

x1, y1, z1 = 21, 21, 30
# Calculate IK angles
theta1, theta2 = inverse_kinematics(x1, y1, z1, L1, L2)
print(math.degrees(theta1), 90-math.degrees(theta2))