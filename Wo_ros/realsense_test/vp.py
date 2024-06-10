import math
import matplotlib.pyplot as plt
import numpy as np
import rrt

def calculate_trapezoidal_profile(path, max_velocity, max_acceleration, fps=10):
    velocities = []
    headings = []
    total_time = 0.0
    current_velocity = 0.0
    time_intervals = []

    for i in range(len(path) - 1):
        start_point = path[i]
        end_point = path[i + 1]
        distance = math.hypot(end_point[0] - start_point[0], end_point[1] - start_point[1])
        if distance == 0:
            continue

        # Calculate heading angle
        heading = math.atan2(end_point[1] - start_point[1], end_point[0] - start_point[0])

        # Calculate the maximum possible velocity at the end of the segment
        acceleration_time = (max_velocity - current_velocity) / max_acceleration
        acceleration_distance = current_velocity * acceleration_time + 0.5 * max_acceleration * acceleration_time ** 2

        if distance < acceleration_distance:
            peak_velocity = math.sqrt(current_velocity ** 2 + 2 * max_acceleration * distance)
            acceleration_time = (peak_velocity - current_velocity) / max_acceleration
            cruise_velocity = peak_velocity
            cruise_time = 0
        else:
            remaining_distance = distance - acceleration_distance
            cruise_velocity = max_velocity
            cruise_time = remaining_distance / max_velocity

        total_segment_time = acceleration_time + cruise_time

        # Calculate acceleration phase
        accel_phase_time_points = np.linspace(total_time, total_time + acceleration_time, int(acceleration_time * fps))
        accel_phase_velocity_points = np.linspace(current_velocity, cruise_velocity, len(accel_phase_time_points)).tolist()
        
        # Calculate cruise phase
        cruise_phase_time_points = np.linspace(total_time + acceleration_time, total_time + total_segment_time, int(cruise_time * fps))
        cruise_phase_velocity_points = [cruise_velocity] * len(cruise_phase_time_points)
        
        velocities.extend([round(v, 2) for v in (accel_phase_velocity_points + cruise_phase_velocity_points)])
        headings.extend([heading] * len(accel_phase_velocity_points + cruise_phase_velocity_points))
        time_intervals.extend(accel_phase_time_points.tolist() + cruise_phase_time_points.tolist())
        total_time += total_segment_time

        current_velocity = cruise_velocity

    # Deceleration phase to stop at the end
    deceleration_time = current_velocity / max_acceleration
    segment_time_points = np.linspace(total_time, total_time + deceleration_time, int(deceleration_time * fps))
    decel_phase_velocity_points = np.linspace(current_velocity, 0, len(segment_time_points)).tolist()
    
    velocities.extend([round(v, 2) for v in decel_phase_velocity_points])
    headings.extend([headings[-1]] * len(segment_time_points))
    time_intervals.extend(segment_time_points.tolist())
    total_time += deceleration_time

    return velocities, headings, time_intervals, total_time

def plot_path_with_velocities_and_headings(path, velocities, headings):
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot([p[0] for p in path], [p[1] for p in path], '-o')
    plt.title('Path')
    plt.xlabel('x')
    plt.ylabel('y')

    plt.subplot(3, 1, 2)
    plt.plot(velocities)
    plt.title('Velocities')
    plt.xlabel('Time step')
    plt.ylabel('Velocity')

    plt.subplot(3, 1, 3)
    plt.plot(headings)
    plt.title('Headings')
    plt.xlabel('Time step')
    plt.ylabel('Heading')

    plt.tight_layout()
    plt.show()
    plt.pause(1)  # Display for 1 second
    plt.close()
    plt.pause(1)  # Display for 1 second
    plt.close()
    plt.pause(1)  # Display for 1 second
    plt.close()

def write_velocities_to_file(velocities, fps, filename="velocities.txt"):
    with open(filename, 'w') as f:
        for velocity in velocities:
            f.write(f"{velocity}\n")
def write_heading_angles_to_file(headings, filename="heading_angle.txt"):
    with open(filename, 'w') as f:
        for heading in headings:
            f.write(f"{heading}\n")

def main(goal_x=6.0,goal_y = 6.0,start_x = 0,start_y = 0):
    import rrt

    

    path = rrt.main(gx=goal_x, gy=goal_y, sx=start_x, sy=start_y)
    print("Generated Path:", path)
    path = path[::-1]

    max_velocity = 0.5  # Maximum velocity
    max_acceleration = 0.1  # Maximum acceleration

    velocities, headings, time_intervals, total_time = calculate_trapezoidal_profile(path, max_velocity, max_acceleration)

    print("Velocities:", velocities)
    print("Headings:", headings)
    print("Total Time:", total_time)

    plot_path_with_velocities_and_headings(path, velocities, headings)

    # Write velocities to a text file at 10 fps
    write_velocities_to_file(velocities, fps=10)
    # Call the function to write heading angles to a file
    write_heading_angles_to_file(headings)


if __name__ == "__main__":
    main()
