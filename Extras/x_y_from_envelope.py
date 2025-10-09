import numpy as np

def xy_for_given_z(z, L1=11.0, L2=36.0, num_points=100):
    if not (L1 - L2 <= z <= L1 + L2):
        raise ValueError("z is out of reach for the arm.")
    theta2 = np.arcsin((z - L1) / L2)
    theta2_options = [theta2, np.pi - theta2]
    theta1_range = np.linspace(-np.pi, np.pi, num_points)
    min_diff = float('inf')
    best_x, best_y = None, None
    for t2 in theta2_options:
        R = L2 * np.cos(t2)
        x = R * np.cos(theta1_range)
        y = R * np.sin(theta1_range)
        diffs = np.abs(x - y)
        idx = np.argmin(diffs)
        if diffs[idx] < min_diff:
            min_diff = diffs[idx]
            best_x, best_y = x[idx], y[idx]
    return best_x, best_y

# Example usage:
z_val = 30.0  # Replace with your desired z
x, y = xy_for_given_z(z_val)
print(f"x: {x}, y: {y}")

# import numpy as np

# def xy_for_given_z(z, L1=11.0, L2=36.0, num_points=100):
#     # Check if z is reachable
#     if not (L1 - L2 <= z <= L1 + L2):
#         raise ValueError("z is out of reach for the arm.")
#     # Solve for theta2
#     theta2 = np.arcsin((z - L1) / L2)
#     # For both possible theta2 (elbow up/down)
#     theta2_options = [theta2, np.pi - theta2]
#     theta1_range = np.linspace(-np.pi, np.pi, num_points)
#     xy_points = []
#     for t2 in theta2_options:
#         R = L2 * np.cos(t2)
#         x = R * np.cos(theta1_range)
#         y = R * np.sin(theta1_range)
#         xy_points.append((x, y))
#     return xy_points  # Returns list of (x, y) arrays for both configurations

# # Example usage:
# z_val = 10.0  # Replace with your desired z
# xy_sets = xy_for_given_z(z_val)
# for i, (x, y) in enumerate(xy_sets):
#     print(f"Configuration {i+1}:")
#     print("x:", x)
#     print("y:", y)