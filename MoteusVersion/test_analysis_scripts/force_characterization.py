import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import pchip_interpolate
import matplotlib as mpl

def process_angle_force_data(file_path):
    # Read the CSV file
    df = pd.read_csv(file_path)
    
    # Group by commanded_cam_angle and calculate mean of measured_force
    angle_force_df = df.groupby('commanded_cam_angle')['measured_force'].agg(['mean', 'count']).reset_index()
    angle_force_df.columns = ['angle', 'avg_force', 'count']
    
    # Sort by angle and filter to max 70 degrees
    angle_force_df = angle_force_df.sort_values('angle')
    angle_force_df = angle_force_df[angle_force_df['angle'] <= 70]
    
    # Define the desired reference curve
    enc_angle = np.array([0, 10, 20, 40, 60, 70, 80])
    desired_force = np.array([0.3, 0.3, 0.3, 0.3, 3, 7, 15])
    
    # Create interpolation points along the entire range
    interp_angles = np.arange(0, 71, 1)  # From 0 to 70 degrees
    interp_forces = pchip_interpolate(enc_angle, desired_force, interp_angles)
    
    # IEEE formatting specifications
    # Standard figure size for IEEE two-column format (3.5 inches wide)
    fig_size = (3.2, 1.8)
    
    # Set up the plot style for IEEE
    plt.style.use('seaborn-v0_8-paper')
    
    # Configure plot parameters for IEEE format
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
    
    # Create the figure
    fig, ax = plt.subplots(figsize=fig_size)
    
    # Plot measured data with smaller markers
    ax.plot(angle_force_df['angle'], angle_force_df['avg_force'], 'o-', 
            color='#fb910f', markersize=4, 
            label='Measured Force')
    
    # Plot the desired reference curve
    ax.plot(interp_angles, interp_forces, '--', color='#953635',
            label='Reference Force')
    
    # Configure the axes
    ax.set_xlabel('Cam Angle (deg)')
    ax.set_ylabel('Force (N)')
    ax.set_xlim(0, 72)
    
    # Add a proper grid
    ax.grid(True, linestyle=':', linewidth=0.3, alpha=0.5)
    
    # Add a legend outside the plot area to save space
    ax.legend(loc='upper left')
    
    # Adjust annotation sizes and positions for IEEE format
    # for i, row in angle_force_df.iterrows():
    #     # Only annotate every other point to avoid crowding
    #     if i % 2 == 0:
    #         ax.annotate(f"{row['avg_force']:.1f}",
    #                   (row['angle'], row['avg_force']),
    #                   textcoords="offset points",
    #                   xytext=(0, 7),
    #                   ha='center',
    #                   fontsize=7)
    
    # Tight layout with specific margins for IEEE
    plt.tight_layout(pad=0.5)
    
    # Save the figure (uncomment when ready)
    plt.savefig('plots/spring_characterization.pdf', 
               dpi=300, 
               bbox_inches='tight',
               pad_inches=0.03)  # Ultra-tight padding
    # plt.savefig('angle_force_ieee.png', format='png', bbox_inches='tight', pad_inches=0.1)
    
    plt.show()
    
    # Calculate error metrics between measured and desired force
    if len(angle_force_df) > 0:
        mse = 0
        mae = 0
        count = 0
        
        print("\nComparison to Reference Force Curve:")
        print("-" * 50)
        print("Angle | Measured | Reference | Difference")
        print("-" * 50)
        
        for i, row in angle_force_df.iterrows():
            angle = row['angle']
            measured = row['avg_force']
            # Find the reference force for this angle
            desired = pchip_interpolate(enc_angle, desired_force, angle)
            diff = measured - desired
            
            mse += diff ** 2
            mae += abs(diff)
            count += 1
            
            print(f"{angle:5.1f} | {measured:8.2f} | {desired:9.2f} | {diff:10.2f}")
        
        if count > 0:
            mse /= count
            mae /= count
            
            print("-" * 50)
            print(f"Mean Squared Error: {mse:.4f}")
            print(f"Mean Absolute Error: {mae:.4f}")
    
    return angle_force_df, interp_angles, interp_forces

# Usage
if __name__ == "__main__":
    # Replace with your actual file path
    root_path = "C:\\Users\\divg2\\OneDrive\\Documents\\GitHub\\SEA-CamActuator\\MoteusVersion\\"
    file_name = r"exo_data\20250502_1013_trans_test_amp20.csv" 
    file_path = root_path + file_name
    angle_force_data, ref_angles, ref_forces = process_angle_force_data(file_path)