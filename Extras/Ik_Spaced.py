import math
import numpy as np
# === Arm Link Lengths ===
L1 = 11.0  # base height
L2 = 36.0  # shoulder link
L3 = 40.0  # elbow link

def inverse_kinematics(x, y, z):
    # --- Step 1: Base rotation (theta1) ---
    theta1 = math.atan2(y, x)   # radians

    # --- Step 2: Projection onto Z-R plane ---
    R = math.sqrt(x**2 + y**2)   # horizontal distance from base
    Z = z - L1                   # vertical distance from base height

    # Distance from shoulder joint to target point
    H = math.sqrt(R**2 + Z**2)

    if H > (L2 + L3) or H < abs(L2 - L3):
        raise ValueError("Target out of reach!")

   
    cos_theta3 = (L2**2 + L3**2 - H**2) / (2 * L2 * L3)
    theta3 = math.acos(cos_theta3)   # radians

    # --- Step 5: Shoulder angle (theta2) ---
    beta = math.atan2(Z, R)
    cos_alpha = (L2**2 + H**2 - L3**2) / (2 * L2 * H)
    alpha = math.acos(cos_alpha) 
    theta2 = beta - alpha   # radians

    # Convert to degrees for readability
    return math.degrees(theta1), math.degrees(theta2), math.degrees(theta3)


if __name__ == "__main__":
    # Ask once
    # x = float(input("Enter target X: "))
    # y = float(input("Enter target Y: "))
    # z = float(input("Enter target Z: "))
    x, y, z = 10, 50, 60
    try:
        t1, t2, t3 = inverse_kinematics(x, y, z)
        print(f"IK Angles : t1={t1:.2f}, t2={t2:.2f}, t3={t3:.2f}")
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
            ax.view_init(elev=0, azim=90, roll=0) # xz plane view
            # ax.view_init(elev=0, azim=0, roll=0) # yz plane view


            plt.legend()
            plt.show()


        plot_robotic_arm(L1, L2, L3, t1, t2, t3)
    except ValueError as e:
        print("Error:", e)
