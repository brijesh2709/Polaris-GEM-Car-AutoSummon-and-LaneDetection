import pandas as pd

# Load the CSV file
file_path = "./xyhead_demo_sim_pp.csv"  # Replace with your actual file path
df = pd.read_csv(file_path, header=None)

# Assign column names manually
df.columns = ['x', 'y', 'yaw']

# Apply the translation
df['x'] = df['x'] + 20
df['y'] = df['y'] + 17.5
# Save the transformed coordinates to a new CSV file
output_file_path = "translated_coordinates.csv"  # Replace with your desired output path
df.to_csv(output_file_path, index=False, header=False)  # Save without headers

print(f"Translated coordinates saved to {output_file_path}")
