import time
import pandas as pd
import matplotlib.pyplot as plt
import os

# Get the latest CSV filename from the text file
with open('curr_datafile_name.txt', 'r') as f:
    csv_filename = f.read().strip()

plt.ion()  # Turn on interactive mode

# Define the time interval for plotting (in seconds)
plot_interval = 0.01  # Plot every 10ms, equivalent to 100Hz plot rate
columns_to_read = ["loop_time", "cam_angle"]

# Initialize the plot
fig, ax = plt.subplots()
line, = ax.plot([], [], linestyle="-", color="b")
ax.set_xlabel("Time")
ax.set_ylabel("Value")
ax.set_title("Real-Time Data Plot (Last 30 Seconds)")
ax.grid(True)
# ax.set_ylim(0, 75)  # Y-axis limit from 0 to 75

# Function to filter data for the last 30 seconds
def filter_last_30_seconds(df, current_time):
    return df[df["loop_time"] >= (current_time - 30)]

try:
    while True:
        try:
            # Read the file and get the new data
            if os.path.exists(csv_filename):  # Ensure the file exists
                # Read all columns
                df = pd.read_csv(csv_filename)
                
                # Filter the columns you need
                df = df[columns_to_read]
                
                # Ensure the columns are numeric
                df["loop_time"] = pd.to_numeric(df["loop_time"])
                df["cam_angle"] = pd.to_numeric(df["cam_angle"])
                
                # Get the current time (last timestamp in the data)
                if not df.empty:
                    current_time = df["loop_time"].iloc[-1]
                    
                    # Filter data for the last 30 seconds
                    filtered_df = filter_last_30_seconds(df, current_time)
                    
                    # Update the plot
                    line.set_xdata(filtered_df["loop_time"])
                    line.set_ydata(filtered_df["cam_angle"])
                    ax.relim()
                    ax.autoscale_view()
                    plt.draw()
                    plt.pause(plot_interval)
                    
        except Exception as e:
            # print(f"Error reading file: {e}")
            continue

        
        time.sleep(plot_interval)  # Sleep to limit the frequency of file access and reduce CPU load

except KeyboardInterrupt:
    print('Ctrl-C detected, Closing')