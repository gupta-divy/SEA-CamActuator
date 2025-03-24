import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import chirp

# File path
csv_file = "20250324_1602_test0.csv"

# User-defined parameters
start_time = 0  
end_time = None 

# Load CSV file
df = pd.read_csv(csv_file)

# Get maximum available time if end_time is not set
if end_time is None:
    end_time = df["loop_time"].max()

# Filter data based on time range
r = 30
df_filtered = df[(df["loop_time"] >= start_time) & (df["loop_time"] <= end_time)] 
# displacement_data = r * np.sin((df_filtered["disturbance_displacement"] * 2 * np.pi)-np.pi/2) + r

first_data = df_filtered["commanded_actuator_velocity"]
second_data = (df_filtered["mc_command_velocity"]/8)*360
# second_data = df_filtered["disturbance_compensation_2"]
third_data = df_filtered["actuator_velocity"]
# second_data = r*2*np.pi*df_filtered["disturbance_velocity_measured"]*np.sin((df_filtered["disturbance_displacement"] * 2 * np.pi))

fourth_data = df_filtered["mc_current"]
fig, ax1 = plt.subplots(figsize=(8, 5))

# Plot cam_angle on left y-axis
ax1.plot(df_filtered["loop_time"], first_data, df_filtered["loop_time"], second_data, df_filtered["loop_time"], third_data)
ax1.set_xlabel("Loop Time (s)")
ax1.tick_params(axis='y', labelcolor="b")
# ax1.set_ylim(-1500, 1500)

# Create right y-axis and apply the alignment shift
ax2 = ax1.twinx()
ax2.plot(df_filtered["loop_time"], fourth_data, linestyle='--')
ax2.tick_params(axis='y', labelcolor="r")
# ax2.set_ylim(-100, 100) 

# Add title and grid
plt.title("Cam Angle and Disturbance Velocity vs. Loop Time")
ax1.grid(True)

# Show the plot
plt.show()
