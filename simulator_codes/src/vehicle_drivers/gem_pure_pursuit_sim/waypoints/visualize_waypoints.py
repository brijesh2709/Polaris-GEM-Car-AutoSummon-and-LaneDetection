import numpy as np
import csv
import matplotlib.pyplot as plt
import pandas as pd
import math
def plot_orig_waypoints(orig, save_path='sensor_waypoints_plot.png'):
    plt.figure(figsize=(12, 8))
    
    # Plot smooth waypoints
    x_smooth = [wp[0] for wp in orig]
    y_smooth = [wp[1] for wp in orig]
    plt.plot(x_smooth, y_smooth, 'b-', label='Sensor Waypoints')
    
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

# df = pd.read_csv(input_csv_file)

# # Calculate the step for downsampling
# original_rows = len(df)
# target_rows = 143
# step = math.floor(original_rows / target_rows) * 2

# # Downsample the DataFrame
# downsampled_df = df.iloc[::step].head(target_rows)

# # Save the downsampled data to a new CSV
# downsampled_file_path = "downsampled_file.csv"  # Replace with your desired output file path
# downsampled_df.to_csv(downsampled_file_path, index=False)

# print(f"Downsampled data saved to {downsampled_file_path}")


plot_orig_waypoints(original_waypoints)

# plot_orig_waypoints(downsampled_df.to_numpy())
