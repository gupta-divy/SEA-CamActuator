import pandas as pd
import matplotlib.pyplot as plt

# File path
csv_file = "20250205_1734_test0.csv"


# User-defined parameters
start_time = 0    # Change to desired start time (in seconds)
end_time = None   # Set to None to use the max available time

# Load CSV file
df = pd.read_csv(csv_file)

# Ensure necessary columns exist
if "loop_time" not in df.columns or "cam_angle" not in df.columns:
    raise ValueError("CSV file must contain 'loop_time' and 'cam_angle' columns.")

# Get maximum available time if end_time is not set
if end_time is None:
    end_time = df["loop_time"].max()

# Filter data based on time range
df_filtered = df[(df["loop_time"] >= start_time) & (df["loop_time"] <= end_time)]

# Create figure and axis
fig, ax1 = plt.subplots(figsize=(8, 5))

# Plot cam_angle on the left y-axis
# ax1.plot(df_filtered["loop_time"], df_filtered["cam_velocity"], linestyle="-", color="b", label="Cam Angle")
# df_filtered["loop_time"], df_filtered["cam_velocity"],df_filtered["loop_time"], df_filtered["disturbance_velocity"]
ax1.plot(df_filtered["loop_time"], df_filtered["cam_angle"],df_filtered["loop_time"], df_filtered["cam_velocity"],df_filtered["loop_time"], df_filtered["disturbance_velocity"])
ax1.set_xlabel("Loop Time (s)")
ax1.set_ylabel("Cam Angle", color="b")
ax1.tick_params(axis='y', labelcolor="b")

# Create a second y-axis for loop_time
ax2 = ax1.twinx()
ax2.plot(df_filtered["loop_time"], df_filtered["actuator_velocity"], linestyle="--", color="r", label="Actuator Velocity")
ax2.set_ylabel("Actuator Velocity", color="r")
ax2.tick_params(axis='y', labelcolor="r")


# Add title and grid
plt.title("Cam Angle & Actuator Velocity vs. Loop Time")
ax1.grid(True)

# Show the plot
plt.show() 