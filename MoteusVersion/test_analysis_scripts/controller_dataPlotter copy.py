import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import chirp

# File path
root_path = "C:\\Users\\divg2\\OneDrive\\Documents\\GitHub\\SEA-CamActuator\\MoteusVersion\\"
file_name = r"exo_data\20250428_1246_no_mechanism_high_slack.csv"
file_name_2 = r"exo_data\20250428_1132_active.csv"
csv_file = root_path+file_name
csv_file2 = root_path+file_name_2

df = pd.read_csv(csv_file)
df2 = pd.read_csv(csv_file2)
# User-defined parameters
start_time = df.loc[df['commanded_actuator_torque'].notna(), 'loop_time'].iloc[0]
end_time = start_time+2
start_time2 = df2.loc[df2['commanded_cable_force']==30, 'loop_time'].iloc[0]
end_time2 = start_time2+2
print(start_time2, end_time2)
# Load CSV file

# Get maximum available time if end_time is not set
if end_time is None:
    end_time = df["loop_time"].max()

# Filter data based on time range
r = 20
df_filtered = df[(df["loop_time"] >= start_time) & (df["loop_time"] <= end_time)]
df2_filtered =  df2[(df2["loop_time"] >= start_time2) & (df2["loop_time"] <= end_time2)]
# displacement_data = r * np.sin((df_filtered["disturbance_displacement"] * 2 * np.pi)-np.pi/2) + r

# first_data = df_filtered["disturbance_velocity"]
# second_data = r*2*np.pi*df_filtered["measured_disturbance_velocity"]*np.sin((df_filtered["measured_disturbance_displacement"] * 2 * np.pi))
# first_data = (df_filtered["mc_command_velocity"]/8)*360
# second_data = df_filtered["actuator_velocity"]
# first_data = df_filtered["commanded_cam_angle"]
first_data = df_filtered["commanded_actuator_torque"]/0.032
second_data = df_filtered["measured_force"]

# second_data = df_filtered["cam_angle"]
# third_data = df_filtered["actuator_angle"]
third_data = df2_filtered["commanded_cable_force"]
fourth_data = df2_filtered["measured_force"]

# Data variables
x_data = df_filtered["loop_time"] - df_filtered['loop_time'].iloc[0]  # X-axis data
y1_data = first_data              # Left y-axis primary data
y2_data = second_data             # Left y-axis secondary data
y3_data = third_data              # Right y-axis data


# ===== USER-CONFIGURABLE VARIABLES =====
x_data = df_filtered["loop_time"] - df_filtered["loop_time"].iloc[0]  # Reset to start at 0
x_data2 = df2_filtered["loop_time"] - df2_filtered["loop_time"].iloc[0]  # Reset to start at 0
y1_data = first_data
y2_data = second_data
y3_data = third_data

x_label = "Loop Time (s)"
y_label_l = "Cable_force"
y_label_r = "Cable_force"

y2_label = "Measured wo mech"
y3_label = "Commanded Force"
y4_label = 'Measured w mech'

line_colors = {'y1': '#1f77b4', 'y2': '#ff7f0e', 'y3': '#d62728'}
line_styles = {'y1': '-', 'y2': '-', 'y3': '-'}

y1_limits = (-1, 50)
y3_limits = (-1, 50)
x_limits = (10, 35)
fig_size = (3.2, 1.8)

# ===== PLOT SETUP =====
plt.style.use('seaborn-v0_8-paper')
plt.rcParams.update({
    'font.family': 'Arial',
    'font.size': 5,
    'axes.labelsize': 6,
    'xtick.labelsize': 5,
    'ytick.labelsize': 5,
    'legend.fontsize': 6,
    'axes.linewidth': 0.6,
    'grid.linewidth': 0.4,
    'lines.linewidth': 1.0,
    'xtick.major.pad': 2,
    'ytick.major.pad': 2,
})

fig, ax1 = plt.subplots(figsize=fig_size)

# ===== LEFT AXIS =====
# ax1.plot(x_data, y1_data, color=line_colors['y1'], linestyle=line_styles['y1'], label=y1_label)
ax1.plot(x_data, y2_data, color=line_colors['y2'], linestyle=line_styles['y2'], label=y2_label)
ax1.plot(x_data2, y3_data, color='r', linestyle=line_styles['y3'], label=y3_label)
ax1.plot(x_data2, fourth_data, color='g', linestyle=line_styles['y2'], label=y4_label)
ax1.legend()
ax1.set_xlabel(x_label, fontweight='normal', labelpad=1)  # Reduced label padding
ax1.set_ylabel(y_label_l, fontweight='normal')

ax1.tick_params(axis='y', labelcolor=line_colors['y1'], rotation=45, pad=1)
# ax1.set_ylim(y1_limits)

# ax1.set_xlim(x_limits)
ax1.grid(True, linestyle=':', linewidth=0.3, alpha=0.5)
ax1.yaxis.set_label_coords(-0.075, 0.5)  # Move y-label closer

# # # ===== RIGHT AXIS =====
# ax2 = ax1.twinx()  # This correctly creates the second axis
# ax2.plot(x_data2, y3_data, color='r', linestyle=line_styles['y3'], label=y3_label)
# ax2.plot(x_data2, fourth_data, color='g', linestyle=line_styles['y2'], label=y4_label)
# # ax2.tick_params(axis='y', labelcolor=line_colors['y3'], rotation=45, pad=1)
# # ax2.set_ylabel(y_label_r, fontweight='normal')
# ax2.set_ylim(y3_limits)
# # ax2.yaxis.set_label_coords(1.075, 0.5)  # Position right y-label

# ## ===== LEGEND =====
# lines1, labels1 = ax1.get_legend_handles_labels()
# lines2, labels2 = ax2.get_legend_handles_labels()
# ax1.legend(lines1 + lines2, labels1 + labels2,
#           loc='upper right',
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

# plt.savefig('plots/dist_vs_camAngle.pdf', 
#            dpi=300, 
#            bbox_inches='tight',
#            pad_inches=0.03)  # Ultra-tight padding
plt.show()