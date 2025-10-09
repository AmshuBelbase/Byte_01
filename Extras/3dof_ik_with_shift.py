import numpy as np 
import math

# Link lengths in cm
L1 = 5.995  # Link 1 length
L2 = 22  # Link 2 length
L3 = 21.5  # Link 3 length
linkConst = 9.094  # Constant link between L1 and L2

# IK function: calculates joint angles theta1 (base twist), theta2 and theta3 with linkConst adjustment   
def inverse_kinematics(x, y, z, L1, linkConst, L2, L3):
    theta1 = np.arctan2(y, x)  # Base twist angle
 
    shift_x = linkConst * np.cos(theta1+math.radians(45))
    shift_y = linkConst * np.sin(theta1+math.radians(45))

    # print(shift_x, shift_y)
    
    # Adjust x coordinate relative to end of linkConst
    x_adj = x - shift_x
    y_adj = y - shift_y
    z_adj = z - L1
    
    print(x_adj, y_adj, z_adj)
 
    # Distance from linkConst end to target
    L = np.sqrt(x_adj**2 + y_adj**2 + (z_adj)**2)
    
    # lower-leg angle using law of cosines
    theta3 = -np.arccos((L**2 - L2**2 - L3**2) / (2 * L2 * L3))
    
    # upper-leg angle calculation
    beta = np.arctan2(z_adj, np.sqrt(x_adj**2 + y_adj**2)) 
    alpha = np.arctan2(L3 * np.sin(theta3), L2 + (L3 * np.cos(theta3)))
    theta2 = beta - alpha
    
    # reach = np.sqrt((z_adj)**2 + x_adj**2 + y_adj**2)
    # if reach > L2 + L3 + 1e-9:
    #     raise ValueError("Target out of reach for links L2 and L3") 
    
    return theta1, theta2, theta3


x_home, y_home, z_home = 0, 9.094, 49.495 # Home position (x,y,z) in cm

# theta1_home, theta2_home, theta3_home = 0, 90, 0

# x, y, z = -21.5 , 0, -21.5
# Example target position (x,y,z) in cm
# x1, y1, z1 = x - x_home, y - y_home, z - z_home
x1, y1, z1 = 0, 9.094, 49.495

# print(x1, y1, z1)
# x1, y1, z1 = 0, 9.094, 49.495

# Calculate IK angles
theta1, theta2, theta3 = inverse_kinematics(x1, y1, z1, L1, linkConst, L2, L3)

# theta1, theta2, theta3 = math.radians(45), math.radians(0), math.radians(0)
# print(math.degrees(theta1), math.degrees(theta2), math.degrees(theta3)) 
# print(math.degrees(theta1)-theta1_home, math.degrees(theta2)-theta2_home, math.degrees(theta3)-theta3_home) 
 
import matplotlib.pyplot as plt 

def plot_robotic_arm(L1, linkConst, L2, L3, theta1, theta2, theta3):
    # Base origin
    O = np.array([0, 0, 0])

    # First joint above base is vertical link L1
    J1 = np.array([0, 0, L1])

    shift_x = linkConst * np.cos(theta1 + math.radians(90))
    shift_y = linkConst * np.sin(theta1 + math.radians(90))

    print(shift_x, shift_y)

    # New constant link extending along x axis from J1
    J_const = J1 + np.array([shift_x, shift_y, 0])

    # Second joint position (shoulder) relative to J_const using shoulder angle theta2 and base twist theta1
    J2 = J_const + np.array([
        L2 * np.cos(theta2) * np.cos(theta1),
        L2 * np.cos(theta2) * np.sin(theta1),
        L2 * np.sin(theta2)
    ])

    # End effector (third joint) position using sum of shoulder and elbow angles theta2 + theta3, relative to J2
    J3 = J2 + np.array([
        L3 * np.cos(theta2 + theta3) * np.cos(theta1),
        L3 * np.cos(theta2 + theta3) * np.sin(theta1),
        L3 * np.sin(theta2 + theta3)
    ])

    max_reach = L1 + linkConst + L2 + L3
    
    fig = plt.figure(figsize=(12, 12))

    # YZ plane view (azim=0)
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.plot([O[0], J1[0]], [O[1], J1[1]], [O[2], J1[2]], 'k-', lw=4, label='L1')
    ax1.plot([J1[0], J_const[0]], [J1[1], J_const[1]], [J1[2], J_const[2]], 'g-', lw=4, label='Lc')
    ax1.plot([J_const[0], J2[0]], [J_const[1], J2[1]], [J_const[2], J2[2]], 'b-', lw=4, label='L2')
    ax1.plot([J2[0], J3[0]], [J2[1], J3[1]], [J2[2], J3[2]], 'r-', lw=4, label='L3')
    ax1.scatter([O[0], J1[0], J_const[0], J2[0], J3[0]], 
                [O[1], J1[1], J_const[1], J2[1], J3[1]], 
                [O[2], J1[2], J_const[2], J2[2], J3[2]], color='k', s=50)
    ax1.set_xlabel('X (cm)')
    ax1.set_ylabel('Y (cm)')
    ax1.set_zlabel('Z (cm)')
    ax1.set_title('YZ Plane View')
    ax1.set_box_aspect([1,1,1])
    ax1.set_xlim([-max_reach, max_reach])
    ax1.set_ylim([-max_reach, max_reach])
    ax1.set_zlim([0, max_reach])
    ax1.view_init(elev=0, azim=0, roll=0)
    ax1.legend()

    # XZ plane view (azim=-90)
    ax2 = fig.add_subplot(2,2,2, projection='3d')
    ax2.plot([O[0], J1[0]], [O[1], J1[1]], [O[2], J1[2]], 'k-', lw=4, label='L1')
    ax2.plot([J1[0], J_const[0]], [J1[1], J_const[1]], [J1[2], J_const[2]], 'g-', lw=4, label='Lc')
    ax2.plot([J_const[0], J2[0]], [J_const[1], J2[1]], [J_const[2], J2[2]], 'b-', lw=4, label='L2')
    ax2.plot([J2[0], J3[0]], [J2[1], J3[1]], [J2[2], J3[2]], 'r-', lw=4, label='L3')
    ax2.scatter([O[0], J1[0], J_const[0], J2[0], J3[0]], 
                [O[1], J1[1], J_const[1], J2[1], J3[1]], 
                [O[2], J1[2], J_const[2], J2[2], J3[2]], color='k', s=50)
    ax2.set_xlabel('X (cm)')
    ax2.set_ylabel('Y (cm)')
    ax2.set_zlabel('Z (cm)')
    ax2.set_title('XZ Plane View')
    ax2.set_box_aspect([1,1,1])
    ax2.set_xlim([-max_reach, max_reach])
    ax2.set_ylim([-max_reach, max_reach])
    ax2.set_zlim([0, max_reach])
    ax2.view_init(elev=0, azim=-90, roll=0)
    ax2.legend()

    # XY plane view (azim=90)
    ax3 = fig.add_subplot(2,2,3, projection='3d')
    ax3.plot([O[0], J1[0]], [O[1], J1[1]], [O[2], J1[2]], 'k-', lw=4, label='L1')
    ax3.plot([J1[0], J_const[0]], [J1[1], J_const[1]], [J1[2], J_const[2]], 'g-', lw=4, label='Lc')
    ax3.plot([J_const[0], J2[0]], [J_const[1], J2[1]], [J_const[2], J2[2]], 'b-', lw=4, label='L2')
    ax3.plot([J2[0], J3[0]], [J2[1], J3[1]], [J2[2], J3[2]], 'r-', lw=4, label='L3')
    ax3.scatter([O[0], J1[0], J_const[0], J2[0], J3[0]], 
                [O[1], J1[1], J_const[1], J2[1], J3[1]], 
                [O[2], J1[2], J_const[2], J2[2], J3[2]], color='k', s=50)
    ax3.set_xlabel('X (cm)')
    ax3.set_ylabel('Y (cm)')
    ax3.set_zlabel('Z (cm)')
    ax3.set_title('XY Plane View')
    ax3.set_box_aspect([1,1,1])
    ax3.set_xlim([-max_reach, max_reach])
    ax3.set_ylim([-max_reach, max_reach])
    ax3.set_zlim([0, max_reach])
    ax3.view_init(elev=90, azim=90, roll=90)
    ax3.legend()

    # Isometric plane view (azim=90)
    ax4 = fig.add_subplot(2,2,4, projection='3d')
    ax4.plot([O[0], J1[0]], [O[1], J1[1]], [O[2], J1[2]], 'k-', lw=4, label='L1')
    ax4.plot([J1[0], J_const[0]], [J1[1], J_const[1]], [J1[2], J_const[2]], 'g-', lw=4, label='Lc')
    ax4.plot([J_const[0], J2[0]], [J_const[1], J2[1]], [J_const[2], J2[2]], 'b-', lw=4, label='L2')
    ax4.plot([J2[0], J3[0]], [J2[1], J3[1]], [J2[2], J3[2]], 'r-', lw=4, label='L3')
    ax4.scatter([O[0], J1[0], J_const[0], J2[0], J3[0]], 
                [O[1], J1[1], J_const[1], J2[1], J3[1]], 
                [O[2], J1[2], J_const[2], J2[2], J3[2]], color='k', s=50)
    ax4.set_xlabel('X (cm)')
    ax4.set_ylabel('Y (cm)')
    ax4.set_zlabel('Z (cm)')
    ax4.set_title('Isometric View')
    ax4.set_box_aspect([1,1,1])
    ax4.set_xlim([-max_reach, max_reach])
    ax4.set_ylim([-max_reach, max_reach])
    ax4.set_zlim([0, max_reach])
    ax4.view_init(elev=25, azim=25, roll=0)
    ax4.legend()

    plt.tight_layout()
    plt.show()



# def plot_robotic_arm(L1, linkConst, L2, L3, theta1, theta2, theta3):
    # Base origin
    # O = np.array([0, 0, 0])


    
    # # First joint above base is vertical link L1
    # J1 = np.array([0, 0, L1])

    # shift_x = linkConst * np.cos(theta1)
    # shift_y = linkConst * np.sin(theta1)
    
    # # New constant link extending along x axis from J1
    # J_const = J1 + np.array([shift_x, shift_y, 0])
    
    # # Second joint position (shoulder) relative to J_const using shoulder angle theta2 and base twist theta1
    # J2 = J_const + np.array([
    #     L2 * np.cos(theta2) * np.sin(theta1),
    #     L2 * np.cos(theta2) * np.cos(theta1),
    #     L2 * np.sin(theta2)
    # ])
    
    # # End effector (third joint) position using sum of shoulder and elbow angles theta2 + theta3, relative to J2
    # J3 = J2 + np.array([
    #     L3 * np.cos(theta2 + theta3) * np.sin(theta1),
    #     L3 * np.cos(theta2 + theta3) * np.cos(theta1),
    #     L3 * np.sin(theta2 + theta3)
    # ])
    
    # # Plotting
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    
    # # Plot each link
    # ax.plot([O[0], J1[0]], [O[1], J1[1]], [O[2], J1[2]], 'k-', lw=4, label='Link 1')
    # ax.plot([J1[0], J_const[0]], [J1[1], J_const[1]], [J1[2], J_const[2]], 'g-', lw=4, label='Constant Link')
    # ax.plot([J_const[0], J2[0]], [J_const[1], J2[1]], [J_const[2], J2[2]], 'b-', lw=4, label='Link 2')
    # ax.plot([J2[0], J3[0]], [J2[1], J3[1]], [J2[2], J3[2]], 'r-', lw=4, label='Link 3')
    
    # # Plot joints
    # ax.scatter([O[0], J1[0], J_const[0], J2[0], J3[0]], 
    #            [O[1], J1[1], J_const[1], J2[1], J3[1]], 
    #            [O[2], J1[2], J_const[2], J2[2], J3[2]], color='k', s=50)
    
    # # Labels and title
    # ax.set_xlabel('X (cm)')
    # ax.set_ylabel('Y (cm)')
    # ax.set_zlabel('Z (cm)')
    # ax.set_title('3D Robotic Dog Leg Skeleton Visualization')
    
    # # Equal aspect ratio
    # ax.set_box_aspect([1, 1, 1])
    
    # # Set limits based on max reach (sum of all links)
    # max_reach = L1 + linkConst + L2 + L3
    # ax.set_xlim([-max_reach, max_reach])
    # ax.set_ylim([-max_reach, max_reach])
    # ax.set_zlim([0, max_reach]) 
    # # ax.view_init(elev=-9, azim=-30, roll=-90)
    # # ax.view_init(elev=0, azim=-90, roll=0) # xz plane view
    # ax.view_init(elev=0, azim=0, roll=0) # yz plane view

    
    # plt.legend()
    # plt.show()



plot_robotic_arm(L1, linkConst, L2, L3, theta1, theta2, theta3)
