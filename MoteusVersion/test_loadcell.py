import time
import traceback
import nidaqmx
from nidaqmx.constants import TerminalConfiguration

# Load Cell Calibration Parameters (determined through calibration)
CALIBRATION_FACTOR = 2375
TARE_VALUE = None
measured_force = None

# Create a global task for the DAQ
daq_task = None

# Function to initialize the DAQ
def initialize_daq():
    global daq_task, TARE_VALUE
    
    # Create a new task for analog input
    daq_task = nidaqmx.Task()
    
    # Configure AI0 channel in RSE (Referenced Single-Ended) mode
    daq_task.ai_channels.add_ai_voltage_chan("Dev1/ai0", 
                                          terminal_config=TerminalConfiguration.RSE,
                                          min_val=-5.0, 
                                          max_val=10.0)
    
    # Set the sample rate (similar to Phidget's data interval)
    daq_task.timing.cfg_samp_clk_timing(rate=1000)  # 100 Hz
    
    # Take initial reading for tare
    TARE_VALUE = daq_task.read()
    print(f"TARE Value Set: {TARE_VALUE:.6f}")

# Function to read force from DAQ
def read_force():
    global daq_task, measured_force, measured_mass, measured_force1
    
    if daq_task is not None:
        try:
            # Read voltage from DAQ
            voltage_ratio = daq_task.read() - TARE_VALUE# Read voltage in volts (20v reference)
            force_lb = voltage_ratio * 50/10
            measured_force = 4.44822*force_lb # Convert to force in Newtons
            measured_mass = measured_force*1000 / 9.81 # Convert to mass in g

            # Calculate force using the same calibration approach
            # adjusted_ratio = voltage_ratio
            measured_force1 = (voltage_ratio * CALIBRATION_FACTOR + 2) * 9.81 / 1000
            
        except Exception as e:
            print(f"Error reading DAQ: {e}")

# Main Moteus Controller Code
def main():
    print('Start!')

    
    t0 = time.perf_counter()
    last_actuation_time = t0
    last_print_time = t0
    last_force_read_time = t0

    while True:
        try:
            while time.perf_counter() - last_actuation_time < 0.01:
                pass
            time_now = time.perf_counter()
            
            # Read force at regular intervals
            # if time_now - last_force_read_time >= 0.005:  # Read at 100Hz
            read_force()
                # last_force_read_time = time_now
                
            
            last_actuation_time = time_now
            loop_time = time_now - t0 
            
            
            if time_now - last_print_time >= 1:
                print("Loop Time: ", loop_time,"Force: ", (measured_force1-measured_force)/measured_force*100,"mass: ", measured_mass, "g")
                last_print_time = time_now

        except KeyboardInterrupt:
            print('Ctrl-C detected, Just Quitting')
            break

        except Exception as err:
            print(traceback.print_exc())
            print("Unexpected error: ", err)
            break


    if daq_task is not None:
        daq_task.close()
    print('Done')

if __name__ == "__main__":
    try:
        # Initialize DAQ instead of Phidget
        initialize_daq()
        
        # Run the main async function
        main()

    except KeyboardInterrupt:
        print("Program interrupted.")
        if daq_task is not None:
            daq_task.close()

    except Exception as e:
        print(f"Unexpected error: {e}")
        if daq_task is not None:
            daq_task.close()