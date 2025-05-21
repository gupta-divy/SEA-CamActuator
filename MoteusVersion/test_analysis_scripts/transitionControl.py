import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

time_window = 0.6

# File path
root_path = "C:\\Users\\divg2\\OneDrive\\Documents\\GitHub\\SEA-CamActuator\\MoteusVersion\\"
file_name = r"exo_data\20250428_1246_no_mechanism_more_slack.csv"
file_name_2 = r"exo_data\20250428_1259_no_mechanism_low_slack.csv"
file_name_3 = r"exo_data\20250428_1132_active.csv"
csv_file = root_path + file_name
csv_file2 = root_path + file_name_2
csv_file3 = root_path + file_name_3

# Load CSV files
df = pd.read_csv(csv_file)
df2 = pd.read_csv(csv_file2)
df3 = pd.read_csv(csv_file3)

# Extract loop_time values where commanded_actuator_torque is not NaN
valid_times = df.loc[df['commanded_actuator_torque'].notna(), 'loop_time'].values
df_t0 = valid_times[0]
df_t0_end = df_t0 + time_window
df_t1 = valid_times[np.where(np.diff(valid_times) > 1)[0][0] + 1]
df_t1_end = df_t1 + time_window

# Extract loop_time values where commanded_actuator_torque is not NaN
valid_times = df2.loc[df2['commanded_actuator_torque'].notna(), 'loop_time'].values
print(valid_times)
print(np.where(np.diff(valid_times) > 0.5))
df2_t0 = valid_times[0]
df2_t0_end = df2_t0 + time_window
df2_t1 = valid_times[np.where(np.diff(valid_times) > 1)[0][0] + 1]
df2_t1_end = df2_t1 + time_window

df3_t0 = df3.loc[df3['commanded_cable_force'] == 30, 'loop_time'].iloc[0]
df3_t0_end = df3_t0 + time_window

# Filter each segment
df_filt = df[(df["loop_time"] >= df_t0) & (df["loop_time"] <= df_t0_end)]
df_filt2 = df[(df["loop_time"] >= df_t1) & (df["loop_time"] <= df_t1_end)]
df2_filt = df2[(df2["loop_time"] >= df2_t0) & (df2["loop_time"] <= df2_t0_end)]
df2_filt2 = df2[(df2["loop_time"] >= df2_t1) & (df2["loop_time"] <= df2_t1_end)]
df3_filt = df3[(df3["loop_time"] >= df3_t0) & (df3["loop_time"] <= df3_t0_end)]

# Extract data for each segment
x_df_t0 = df_filt["loop_time"] - df_filt["loop_time"].iloc[0]
x_df_t1 = df_filt2["loop_time"] - df_filt2["loop_time"].iloc[0]
x_df2_t0 = df2_filt["loop_time"] - df2_filt["loop_time"].iloc[0]
x_df2_t1 = df2_filt2["loop_time"] - df2_filt2["loop_time"].iloc[0]
x_df3_t0 = df3_filt["loop_time"] - df3_filt["loop_time"].iloc[0]


# Extract data for each segment
y_df_t0 = df_filt["measured_force"]
y_df_t1 = df_filt2["measured_force"]
y_df2_t0 = df2_filt["measured_force"]
y_df2_t1 = df2_filt2["measured_force"]
y_df3_t0 = df3_filt["measured_force"]

# Labels
x_label = "Loop Time (s)"
y1 = "10mm slack"
y2 = "20mm slack"
y3 = "1mm slack"
y4 = "5mm slack"
y5 = "Active Control"

# Line styles and colors
line_colors = {
    'y1': '#808080',
    'y2': '#5e5e5e',
    'y3': '#d3d3d3',
    'y4': '#a9a9a9',
    'y5': "#fb910f" 
}
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
line_styles = {'solid': '-', 'dashed': '--'}

# Plot settings
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

fig, ax1 = plt.subplots(figsize=(3.2, 1.8))

# Plot data
ax1.plot(x_df2_t0, y_df2_t0, color=line_colors['y3'], linestyle=line_styles['solid'], label=y3)
ax1.plot(x_df2_t1, y_df2_t1, color=line_colors['y4'], linestyle=line_styles['solid'], label=y4)
ax1.plot(x_df_t0, y_df_t0, color=line_colors['y1'], linestyle=line_styles['solid'], label=y1)
ax1.plot(x_df_t1, y_df_t1, color=line_colors['y2'], linestyle=line_styles['solid'], label=y2)
ax1.plot(x_df3_t0, y_df3_t0, color=line_colors['y5'], linestyle=line_styles['solid'], label=y5)
ax1.axhline(y=30, color='#953635', linestyle='--')
ax1.set_xlim((0,0.5))

# Labels and grid
ax1.set_xlabel(x_label, fontweight='normal', labelpad=1)
ax1.set_ylabel("Force (N)", fontweight='normal')
ax1.tick_params(axis='y', labelcolor='black', pad=1)
ax1.grid(True, linestyle=':', linewidth=0.3, alpha=0.5)
ax1.yaxis.set_label_coords(-0.125, 0.5)

# Legend
ax1.legend(loc='upper right', frameon=True, framealpha=1, edgecolor='0.8', handlelength=1.5)

# Layout
plt.subplots_adjust(left=0.15, right=0.85, bottom=0.15, top=0.92)

# Save or show
plt.savefig('plots/transition_control.pdf', dpi=300, bbox_inches='tight', pad_inches=0.03)
plt.show()
