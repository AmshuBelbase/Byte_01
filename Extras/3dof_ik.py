import numpy as np 
import math

# Link lengths in cm
L1 = 11  # Link 1 length
L2 = 36  # Link 2 length
L3 = 40  # Link 3 length

# IK function: calculates joint angles theta1 (base twist), theta2 and theta3
def inverse_kinematics(x, y, z, L1, L2, L3):
    theta1 = np.arctan2(y, x)  # Base twist angle
    L = np.sqrt(x**2 + y**2 + (z-L1)**2)   # Distance from base axis horizontally
    theta3 = np.arccos((L**2 - L2**2 - L3**2) / (2 * L2 * L3))  # Elbow angle
    beta = np.arctan2(z-L1, np.sqrt(x**2 + y**2))
    alpha = np.arctan2(L3 * np.sin(theta3), L2 + L3 * np.cos(theta3))
    theta2 = beta - alpha  # Shoulder angle
    reach = np.sqrt((z - L1)**2 + x**2 + y**2)
    if reach > L2 + L3 + 1e-9:
        raise ValueError("Target out of reach for links L2 and L3")
    return theta1, theta2, theta3

# Example target position (x,y,z) in cm
x1, y1, z1 = 10, 50, 60
# Calculate IK angles
theta1, theta2, theta3 = inverse_kinematics(x1, y1, z1, L1, L2, L3)
print(math.degrees(theta1), 90-math.degrees(theta2), math.degrees(theta3)) 
 
import matplotlib.pyplot as plt

def plot_robotic_arm(L1, L2, L3, theta1, theta2, theta3):
    # Calculate joint positions
    O = np.array([0, 0, 0])  # Base origin
    J1 = np.array([0, 0, L1])  # First joint above base

    # Second joint position calculated using shoulder angle (theta2) and base twist (theta1)
    J2 = J1 + np.array([
        L2 * np.cos(theta2) * np.cos(theta1),
        L2 * np.cos(theta2) * np.sin(theta1),
        L2 * np.sin(theta2)
    ])

    # End effector position using sum of shoulder and elbow angles (theta2 + theta3)
    J3 = J2 + np.array([
        L3 * np.cos(theta2 + theta3) * np.cos(theta1),
        L3 * np.cos(theta2 + theta3) * np.sin(theta1),
        L3 * np.sin(theta2 + theta3)
    ])

    # Plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot links
    ax.plot([O[0], J1[0]], [O[1], J1[1]], [O[2], J1[2]], 'k-', lw=4, label='Link 1')
    ax.plot([J1[0], J2[0]], [J1[1], J2[1]], [J1[2], J2[2]], 'b-', lw=4, label='Link 2')
    ax.plot([J2[0], J3[0]], [J2[1], J3[1]], [J2[2], J3[2]], 'r-', lw=4, label='Link 3')

    # Plot joints
    ax.scatter([O[0], J1[0], J2[0], J3[0]], [O[1], J1[1], J2[1], J3[1]], [O[2], J1[2], J2[2], J3[2]], color='k', s=50)

    # Set labels and title
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_zlabel('Z (cm)')
    ax.set_title('3D Robotic Arm Visualization')

    # Equal aspect ratio
    ax.set_box_aspect([1, 1, 1])

    # Set limits to visualize the full arm reach nicely
    max_reach = L1 + L2 + L3
    ax.set_xlim([-max_reach, max_reach])
    ax.set_ylim([-max_reach, max_reach])
    ax.set_zlim([0, max_reach])
    # ax.view_init(elev=0, azim=-90, roll=0) # xz plane view
    ax.view_init(elev=0, azim=0, roll=0) # yz plane view

    plt.legend()
    plt.show()


plot_robotic_arm(L1, L2, L3, theta1, theta2, theta3)
