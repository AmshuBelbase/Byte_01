import numpy as np

def angle_between_points(A, B, r):
    # Calculate angle of points A and B from origin using atan2
    theta_A = np.arctan2(A[1], A[0])  # For A=(r,0), this is 0
    theta_B = np.arctan2(B[1], B[0])
    
    # Calculate angle difference in radians
    delta_theta = theta_B - theta_A
    
    # Normalize angle to range 0 to 2*pi
    if delta_theta < 0:
        delta_theta += 2 * np.pi
    
    # Convert radians to degrees
    angle_deg = np.degrees(delta_theta)
    return angle_deg

r = 10
A = (r, 0)
B = (-6, -8)
C = (-6, 8)

angle_B = angle_between_points(A, B, r)
angle_C = angle_between_points(A, C, r)

print(f"Angle A to B: {angle_B:.2f} degrees")
print(f"Angle A to C: {angle_C:.2f} degrees")
