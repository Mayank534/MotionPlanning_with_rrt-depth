import numpy as np

def calculate_velocity_and_heading(depth_m, angle_rad):
    # Calculate target coordinates
    target_x = depth_m * np.cos(angle_rad)
    target_y = depth_m * np.sin(angle_rad)

    # Parameters
    v_max = 0.5     # Maximum velocity in m/s
    a = 0.1         # Acceleration in m/s^2 (can be adjusted)
    stop_distance = 0.5  # Distance to stop before reaching the target

    # Calculate distances
    distance = np.sqrt(target_x**2 + target_y**2)
    effective_distance = distance - stop_distance

    # Calculate time to reach max velocity
    t_acc = v_max / a

    # Calculate distance covered during acceleration and deceleration
    d_acc = 0.5 * a * t_acc**2

    # Calculate distance and time at constant velocity
    d_const = effective_distance - 2 * d_acc
    t_const = d_const / v_max
    
    # Print the values
    print("Target X:", target_x)
    print("Target Y:", target_y)
    print("Velocity Profile:")
    print("Acceleration Time (s):", t_acc)
    print("Constant Velocity Time (s):", t_const)
    print("Total Time (s):", 2 * t_acc + t_const)
