import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV data
data = pd.read_csv('/home/barat/Debojit_WS/Bi-Manual_Redundancy_Work/src/data/joint_angles.csv')

# Extract the Velocity Manipulability column
manipulability = data['Velocity_Manipulability']

# Smooth the data using a moving average with a specified window size
window_size = 10  # Adjust the window size for more or less smoothing
smoothed_manipulability = manipulability.rolling(window=window_size, center=True).mean()

# Plot both raw and smoothed manipulability data
plt.figure(figsize=(10, 6))
plt.plot(manipulability, label='Raw Manipulability', marker='o', linestyle='-', alpha=0.5)
plt.plot(smoothed_manipulability, label=f'Smoothed (window size = {window_size})', linewidth=2, color='red')
plt.title('Velocity Manipulability (Raw vs Smoothed)')
plt.xlabel('Sample Index')
plt.ylabel('Manipulability')
plt.legend()
plt.grid(True)
plt.show()
