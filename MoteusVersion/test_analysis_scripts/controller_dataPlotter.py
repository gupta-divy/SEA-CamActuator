import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Apply Savitzky-Golay smoothing
custom_colors = {
    "orange" : "#fb910f",
    "muted_red" : "#f16767",
    "light_pink": "#d8a3a1",
    "deep_red": "#953635",
    "blush": "#efc9c4",
    "darker_red": "#c64f4a",
    "light_gray": "#d3d3d3",
    "medium_gray": "#5e5e5e"
}
# File path
root_path = "C:\\Users\\divg2\\OneDrive\\Documents\\GitHub\\SEA-CamActuator\\MoteusVersion\\"
file_name = r"exo_data\20250428_1246_no_mechanism_high_slack.csv" 
csv_file = root_path + file_name

# Parameters
start_time = 0
end_time = None
 

# Load and filter data
df = pd.read_csv(csv_file)
if end_time is None:
    end_time = df["loop_time"].max()
df_filtered = df[(df["loop_time"] >= start_time) & (df["loop_time"] <= end_time)].copy()

# Handle repeated-value fill (after interpolation)
for col in ["measured_disturbance_velocity", "measured_disturbance_displacement"]:
    repeated = df_filtered[col] == df_filtered[col].shift(1)
    df_filtered.loc[repeated & repeated.shift(1, fill_value=False), col] = np.nan
    df_filtered.loc[:, col] = df_filtered[col].interpolate(method='linear')


# # Create new time grid: 1000 Hz
# original_time = df_filtered["loop_time"].values
# new_time = np.arange(original_time[0], original_time[-1], 0.001)

# # Interpolate all needed columns onto the new time base
# df_filtered_interp = pd.DataFrame({"loop_time": new_time})
# for col in ["measured_disturbance_velocity", "measured_disturbance_displacement", "disturbance_velocity"]:
#     f = interp1d(original_time, df_filtered[col].values, kind='linear', bounds_error=False, fill_value="extrapolate")
#     df_filtered_interp[col] = f(new_time)

# # Now assign this as your main df_filtered
# df_filtered = df_filtered_interp.copy()

# # Filter data based on time range
r = 20
# first_data = r * np.sin((df_filtered["measured_disturbance_displacement"] * 2 * np.pi)-np.pi/2) + r
# first_data = df_filtered["cam_angle"]
# second_data = r*2*np.pi*df_filtered["measured_disturbance_velocity"]*np.sin((df_filtered["measured_disturbance_displacement"] * 2 * np.pi))
# error_data = first_data - second_data
first_data = df_filtered["actuator_torque"]/0.0325
second_data = df_filtered["commanded_actuator_torque"]/0.0325
third_data  = df_filtered["measured_force"]-2

# print(np.mean(error_data), np.median(error_data)) 

# first_data = (df_filtered["mc_command_velocity"]/8)*360
# second_data = df_filtered[""]
# first_data = df_filtered["commanded_cam_angle"]
# first_data = df_filtered["measured"]/0.0325
# second_data = df_filtered["measured_force"]


# second_data = df_filtered["cam_angle"]
# third_data = df_filtered["actuator_angle"]
# third_data = df_filtered["cam_angle"]
# fourth_data = df_filtered["cam_angle"]

# plt.plot(third_data,first_data, marker = 'x')
# plt.plot(third_data,fourth_data, marker = 'o')
# plt.plot(third_data,second_data, marker = '.')


# ===== USER-CONFIGURABLE VARIABLES =====
x_data = df_filtered["loop_time"] - df_filtered["loop_time"].iloc[0]  # Reset to start at 0
y1_data = first_data
y2_data = second_data
y3_data = third_data

x_label = "Loop Time (s)"
y_label_l = "Cam Angle (degrees)"
y_label_r = "Disturbance (mm/sec)"


y1_label = "Measured"
y2_label = "Predicted"
y3_label = "Error"

line_styles = {'y1': '-', 'y2': ':', 'y3': '-'}

y1_limits = (0, 200)
y3_limits = (0, 200)
x_limits = (0, 9.5)
fig_size = (3.2, 1.8)

# ===== PLOT SETUP =====
plt.style.use('seaborn-v0_8-paper')

plt.rcParams.update({
    'font.family': 'sans-serif',
    'font.size': 6,
    'axes.labelsize': 7,
    'xtick.labelsize': 6.5,
    'ytick.labelsize': 6.5,
    'legend.fontsize': 7,
    'axes.linewidth': 0.6,
    'grid.linewidth': 0.4,
    'lines.linewidth': 1,
    'xtick.major.pad': 2,
    'ytick.major.pad': 2,
})

fig, ax1 = plt.subplots(figsize=fig_size)


# # ===== RIGHT AXIS =====
ax2 = ax1.twinx()  # This correctly creates the second axis
ax2.plot(x_data, y3_data, color=custom_colors["light_gray"], linestyle=line_styles['y3'], label=y3_label)
# ax2.plot(x_data, fourth_data)
ax2.tick_params(axis='y', labelcolor=custom_colors["medium_gray"], pad=0.5)
ax2.set_ylabel(y_label_r, fontweight='normal')
ax2.set_ylim(y3_limits)
for label in ax2.get_yticklabels():
    label.set_rotation(45)
    label.set_verticalalignment('baseline')  # Align upward
    # label.set_y(label.get_position()[1]+0.1)  # Slight upward shift
ax2.yaxis.set_label_coords(1.14, 0.5)  # Position right y-label

# ===== LEFT AXIS =====
ax1.plot(x_data, y1_data, color=custom_colors["orange"], linestyle=line_styles['y1'], label=y1_label)
ax1.plot(x_data, y2_data, color=custom_colors["darker_red"], linestyle=line_styles['y2'], label=y2_label)
# ax1.plot(x_data, y3_data, color=custom_colors["medium_gray"], linestyle=line_styles['y3'], label=y3_label)
ax1.set_xlabel(x_label, fontweight='normal', labelpad=1)  # Reduced label padding
# ax1.axhline(y=5,color=custom_colors["orange"], linestyle='--')
# ax1.axhline(y=35,color=custom_colors["orange"], linestyle='--')
ax1.set_ylabel(y_label_l, fontweight='normal')
ax1.tick_params(axis='y',pad=0.5, labelcolor=custom_colors["orange"])
ax1.set_ylim(y1_limits)
# ax1.set_xlim(x_limits)
ax1.grid(True, linestyle=':', linewidth=0.3, alpha=0.5)
from matplotlib.ticker import MultipleLocator

# Example for fewer divisions (e.g., one major grid every 2 units)
# ax1.xaxis.set_major_locator(MultipleLocator(0.05))   # X-axis: every 2 units
# ax1.yaxis.set_major_locator(MultipleLocator(5))
# ax1.legend()
ax1.yaxis.set_label_coords(-0.09, 0.5)  # Move y-label closer
ax1.set_zorder(2)       # Higher z-order = in front
ax1.patch.set_visible(False)  # Hide the background of ax1 so it doesn't cover ax2




# ## ===== LEGEND =====
# lines1, labels1 = ax1.get_legend_handles_labels()
# lines2, labels2 = ax2.get_legend_handles_labels()
# ax1.legend(lines1 + lines2, labels1 + labels2,
#           loc='upper left',
#           frameon=True,
#           framealpha=1,
#           edgecolor='0.8',
#           handlelength=1.5,
#           borderaxespad=0.5)

# ===== FINAL ADJUSTMENTS =====
plt.subplots_adjust(
    left=0.15,    # Tight left margin
    right=0.85,   # Tight right margin
    bottom=0.15,  # Tight bottom margin
    top=0.92      # Tight top margin
)

# plt.savefig('plots/disturbance_rejection.pdf', 
#            dpi=300, 
#            bbox_inches='tight',
#            pad_inches=0.03)  # Ultra-tight padding
plt.show()