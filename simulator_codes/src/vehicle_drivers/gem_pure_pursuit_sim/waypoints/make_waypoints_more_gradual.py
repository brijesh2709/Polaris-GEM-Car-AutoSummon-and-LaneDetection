import numpy as np
import csv
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

def generate_smooth_waypoints2(waypoints, num_points=2000):
    x = np.array([wp[0] for wp in waypoints])
    y = np.array([wp[1] for wp in waypoints])
    yaw = np.array([wp[2] for wp in waypoints])

    # Create parameter t
    t = np.zeros(len(waypoints))
    t[1:] = np.cumsum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))

    # Create cubic spline interpolations
    cs_x = CubicSpline(t, x)
    cs_y = CubicSpline(t, y)
    cs_yaw = CubicSpline(t, yaw)

    # Generate new points
    t_new = np.linspace(0, t[-1], num_points)
    x_new = cs_x(t_new)
    y_new = cs_y(t_new)
    yaw_new = cs_yaw(t_new)

    return list(zip(x_new, y_new, yaw_new))

def generate_smooth_waypoints(waypoints, increment=0.5):
    # Extract x, y, and yaw
    x = np.array([wp[0] for wp in waypoints])
    y = np.array([wp[1] for wp in waypoints])
    yaw = np.array([wp[2] for wp in waypoints])

    # Generate new smoother waypoints
    new_waypoints = []
    for i in range(len(waypoints) - 1):
        start_x, start_y, start_yaw = x[i], y[i], yaw[i]
        end_x, end_y, end_yaw = x[i + 1], y[i + 1], yaw[i + 1]
        distance = np.hypot(end_x - start_x, end_y - start_y)
        num_points = int(distance / increment) + 1

        # Linear interpolation of x, y, and yaw
        new_x = np.linspace(start_x, end_x, num_points)
        new_y = np.linspace(start_y, end_y, num_points)
        new_yaw = np.linspace(start_yaw, end_yaw, num_points)

        # Construct new waypoints
        for nx, ny, nyaw in zip(new_x, new_y, new_yaw):
            new_waypoints.append((nx, ny, nyaw))

    return new_waypoints

def plot_waypoints(smooth, save_path='waypoints_plot.png'):
    plt.figure(figsize=(12, 8))
    
    # Plot smooth waypoints
    x_smooth = [wp[0] for wp in smooth]
    y_smooth = [wp[1] for wp in smooth]
    plt.plot(x_smooth, y_smooth, 'b-', label='Smooth Waypoints')
    
    plt.title('Waypoints Comparison')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')  # Equal aspect ratio

    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Plot saved as {save_path}")

    plt.show()

def plot_orig_waypoints(orig, save_path='orig_waypoints_plot.png'):
    plt.figure(figsize=(12, 8))
    
    # Plot smooth waypoints
    x_smooth = [wp[0] for wp in orig]
    y_smooth = [wp[1] for wp in orig]
    plt.plot(x_smooth, y_smooth, 'b-', label='Original Waypoints')
    
    plt.title('Waypoints in Simulation')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')  # Equal aspect ratio

    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Plot saved as {save_path}")

    plt.show()

# Your original waypoints
# Read original waypoints from CSV file
input_csv_file = './gen_waypoints.csv'
original_waypoints = []

with open(input_csv_file, mode='r') as file:
    csv_reader = csv.reader(file)
    for row in csv_reader:
        original_waypoints.append([float(val) for val in row])

plot_orig_waypoints(original_waypoints)

# Generate smoother waypoints
smooth_waypoints = generate_smooth_waypoints2(original_waypoints)

# Define CSV file name
csv_file = 'smooth_waypoints.csv'

# Write the smooth waypoints to a CSV file
with open(csv_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerows(smooth_waypoints)  # Write data

print(f"Waypoints have been saved to {csv_file}")

plot_waypoints(smooth_waypoints)


