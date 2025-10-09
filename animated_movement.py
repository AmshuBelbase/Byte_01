import _3dof_ik_with_shift_circle_method as ik
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# List of waypoints (replace with your coordinates)
# coords = [(9.094, 38, 5), (9.094, 33, 0), (9.094, 38, -5)]

# Fixed x and linkConst as in your examples
x = 9.094
z_start, z_end = 5, -5
  # The peak deviation - 38
y_start, y_end = 42, 33
y_amplitude = y_start - y_end  # Amplitude of sine wave

num_points = 50  # For smoothness

# Interpolate y from start to end
zs = np.linspace(z_start, z_end, num_points)
# Generate parameter t from 0 to π (for half a sine wave)
ts = np.linspace(0, np.pi, num_points)

# z follows a sine curve with amplitude
ys = y_amplitude * np.sin(ts)

# Build the trajectory points
coords = [(x, y_end-y, z) for y, z in zip(ys, zs)]

fig = plt.figure(figsize=(8,6))
ax = fig.add_subplot(111, projection='3d')
# Link lengths in cm
L1 = 5.995  # Link 1 length
linkConst = 9.094  # Constant link between L1 and L2 (RADIUS OF CIRCLE WHEN L1 IS ROTATED ALONG Z AXIS)
L2 = 22  # Link 2 length
L3 = 21.5  # Link 3 length

def update(num):
    ax.cla()
    x, y, z = coords[num]
    theta1, theta2, theta3 = ik.inverse_kinematics(x, y, z, L1, linkConst, L2, L3)
    ik.visualize.animate_3dof(L1, linkConst, L2, L3, theta1, theta2, theta3, ax=ax)
    ax.set_title(f'Pose {num+1}')

ani = animation.FuncAnimation(fig, update, frames=len(coords), interval=30)
plt.show()
