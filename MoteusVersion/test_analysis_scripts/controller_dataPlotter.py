import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import chirp

# File path
root_path = "C:\\Users\\divg2\\OneDrive\\Documents\\GitHub\\SEA-CamActuator\\MoteusVersion\\"
file_name = r"exo_data\20250404_1025_distTest_amp20.csv"
csv_file = root_path+file_name

# User-defined parameters
start_time = 5  
end_time = 17.5


# Load CSV file
df = pd.read_csv(csv_file)

# Get maximum available time if end_time is not set
if end_time is None:
    end_time = df["loop_time"].max()

# Filter data based on time range
r = 20
df_filtered = df[(df["loop_time"] >= start_time) & (df["loop_time"] <= end_time)] 
# displacement_data = r * np.sin((df_filtered["disturbance_displacement"] * 2 * np.pi)-np.pi/2) + r

first_data = df_filtered["disturbance_velocity"]
second_data = r*2*np.pi*df_filtered["measured_disturbance_velocity"]*np.sin((df_filtered["measured_disturbance_displacement"] * 2 * np.pi))
# first_data = (df_filtered["mc_command_velocity"]/8)*360
# second_data = df_filtered["actuator_velocity"]
# first_data = df_filtered["commanded_cam_angle"]
# second_data = df_filtered["cam_angle"]
third_data = df_filtered["cam_angle"]
# third_data = df_filtered["mc_current"]
# Data variables
x_data = df_filtered["loop_time"] - df_filtered['loop_time'].iloc[0]  # X-axis data
y1_data = first_data              # Left y-axis primary data
y2_data = second_data             # Left y-axis secondary data
y3_data = third_data              # Right y-axis data


# ===== USER-CONFIGURABLE VARIABLES =====
x_data = df_filtered["loop_time"] - df_filtered["loop_time"].iloc[0]  # Reset to start at 0
y1_data = first_data
y2_data = second_data
y3_data = third_data

x_label = "Loop Time (s)"
y_label_l = "Disturbance (mm/sec)"
y_label_r = "Cam Angle (degrees)"

y1_label = "Estimated vel"
y2_label = "Measured vel"
y3_label = "Cam Angle"

line_colors = {'y1': '#1f77b4', 'y2': '#ff7f0e', 'y3': '#d62728'}
line_styles = {'y1': '-', 'y2': '--', 'y3': '-'}

y1_limits = (-1250, 1250)
y3_limits = (0, 40)
x_limits = (0, None)
fig_size = (3.2, 1.8)

# ===== PLOT SETUP =====
plt.style.use('seaborn-v0_8-paper')
plt.rcParams.update({
    'font.family': 'Arial',
    'font.size': 7,
    'axes.labelsize': 8,
    'xtick.labelsize': 7,
    'ytick.labelsize': 7,
    'legend.fontsize': 7,
    'axes.linewidth': 0.6,
    'grid.linewidth': 0.4,
    'lines.linewidth': 1.2,
    'xtick.major.pad': 2,
    'ytick.major.pad': 2,
})

fig, ax1 = plt.subplots(figsize=fig_size)

# ===== LEFT AXIS =====
ax1.plot(x_data, y1_data, color=line_colors['y1'], linestyle=line_styles['y1'], label=y1_label)
ax1.plot(x_data, y2_data, color=line_colors['y2'], linestyle=line_styles['y2'], label=y2_label)
ax1.set_xlabel(x_label, fontweight='normal', labelpad=3)  # Reduced label padding
ax1.set_ylabel(y_label_l, fontweight='normal')
ax1.tick_params(axis='y', labelcolor=line_colors['y1'], rotation=45, pad=1)
ax1.set_ylim(y1_limits)
ax1.set_xlim(x_limits)
ax1.grid(True, linestyle=':', linewidth=0.3, alpha=0.5)
ax1.yaxis.set_label_coords(-0.15, 0.5)  # Move y-label closer

# ===== RIGHT AXIS =====
ax2 = ax1.twinx()  # This correctly creates the second axis
ax2.plot(x_data, y3_data, color=line_colors['y3'], linestyle=line_styles['y3'], label=y3_label)
ax2.tick_params(axis='y', labelcolor=line_colors['y3'], rotation=45, pad=1)
ax2.set_ylabel(y_label_r, fontweight='normal')
ax2.set_ylim(y3_limits)
ax2.yaxis.set_label_coords(1.10, 0.5)  # Position right y-label

# ===== LEGEND =====
lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines1 + lines2, labels1 + labels2,
          loc='upper left',
          frameon=True,
          framealpha=1,
          edgecolor='0.8',
          handlelength=1.5,
          borderaxespad=0.5)

# ===== FINAL ADJUSTMENTS =====
plt.subplots_adjust(
    left=0.15,    # Tight left margin
    right=0.85,   # Tight right margin
    bottom=0.15,  # Tight bottom margin
    top=0.92      # Tight top margin
)

plt.savefig('plots/dist_vs_camAngle.pdf', 
           dpi=300, 
           bbox_inches='tight',
           pad_inches=0.03)  # Ultra-tight padding
plt.show()