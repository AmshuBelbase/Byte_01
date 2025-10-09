import numpy as np
import matplotlib.pyplot as plt

# Arm parameters
L1 = 11.0   # vertical offset
L2 = 36.0  # pitched link length

# Joint ranges
theta1_range = np.linspace(-np.pi, np.pi, 100)  # base rotation
theta2_range = np.linspace(-np.pi/2, np.pi/2, 100)  # pitch range

# Meshgrid for all joint angles
T1, T2 = np.meshgrid(theta1_range, theta2_range)

# Compute workspace points
R = L2 * np.cos(T2)
X = R * np.cos(T1)
Y = R * np.sin(T1)
Z = L1 + L2 * np.sin(T2)

# Plot in 3D
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")
ax.scatter(X, Y, Z, c=Z, cmap="viridis", s=2)

# Labels
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("2-DOF Arm Workspace Envelope")

plt.show()
