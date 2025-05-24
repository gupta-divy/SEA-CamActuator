import ACTUATOR_CODE_MOTEUS
import time
import traceback
import Controllers
from Controllers import SetpointType
import threading
from keyboard_interrupt_parser import ParameterParser
import asyncio
import nidaqmx
from nidaqmx.constants import TerminalConfiguration

FILENAME = "trans_test_amp20"

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
    daq_task.timing.cfg_samp_clk_timing(rate=100)  # 100 Hz
    
    # Take initial reading for tare
    TARE_VALUE = daq_task.read()
    print(f"TARE Value Set: {TARE_VALUE:.6f}")

# Function to read force from DAQ
def read_force():
    global daq_task, measured_force
    
    if daq_task is not None:
        try:
            # Read voltage from DAQ
            voltage_ratio = daq_task.read()
            
            # Calculate force using the same calibration approach
            adjusted_ratio = voltage_ratio

            measured_force = (adjusted_ratio * CALIBRATION_FACTOR + 2) * 9.81 / 1000
            
        except Exception as e:
            print(f"Error reading DAQ: {e}")

# Main Moteus Controller Code
async def main():
    datafile_name = FILENAME
    actuator = await ACTUATOR_CODE_MOTEUS.connect_to_actuator(dataFile_name=datafile_name)
    await actuator.initial_calibration()
    print('Start!')

    # Setup controller
    actuator_controller = Controllers.CommandSetpoint(actuator)

    # Defining controller variables
    target_period = 1 / actuator.config.control_loop_freq

    # Keyboard interrupt initialization
    lock = threading.Lock() 
    quit_event = threading.Event()
    new_setpoint_event = threading.Event()
    gain_event = threading.Event()
    error_tracking = 0
    keyboard_thread = ParameterParser(lock=lock, quit_event=quit_event, new_setpoint_event=new_setpoint_event, gain_event=gain_event)

    t0 = time.perf_counter()
    last_actuation_time = t0
    last_print_time = t0
    last_force_read_time = t0

    while True:
        try:
            while time.perf_counter() - last_actuation_time < target_period:
                pass
            time_now = time.perf_counter()
            
            # Read force at regular intervals
            # if time_now - last_force_read_time >= 0.005:  # Read at 100Hz
            read_force()
                # last_force_read_time = time_now
                
            with lock:
                if new_setpoint_event.is_set():
                    print(f"Updating controller: type={keyboard_thread.setpoint_type}, value={keyboard_thread.setpoint_val}")
                    actuator_controller.update_controller_variables(
                        setpoint_type=keyboard_thread.setpoint_type,
                        setpoint_value=keyboard_thread.setpoint_val,
                    )
                    new_setpoint_event.clear()

                if gain_event.is_set():
                    if keyboard_thread.control_gain_type=='p':
                        actuator.update_camController_gains(kp_gain=keyboard_thread.control_gain)
                    elif keyboard_thread.control_gain_type=='d':
                        actuator.update_camController_gains(kd_gain=keyboard_thread.control_gain)
                    gain_event.clear()

                if quit_event.is_set():
                    break
            last_actuation_time = time_now
            loop_time = time_now - t0 
            await actuator.read_data(loop_time=loop_time)
            actuator.data.measured_force = measured_force
            await actuator_controller.command()
            if time_now - last_print_time >= 1:
                # print("Velocity: ", actuator.data.actuator_velocity, "Cam Angle: ", actuator.data.cam_angle, "Gains", actuator.config.camControllerGainKp, actuator.config.camControllerGainKd)
                print("Torque: ", actuator.data.actuator_torque, 
                      "Commanded: ", actuator.data.commanded_actuator_torque, 
                      "Cam Angle: ", actuator.data.cam_angle, 
                      "Force: ", measured_force)
                last_print_time = time_now
            actuator.write_data()
            if measured_force is not None and measured_force > 200: quit_event.set()

        except KeyboardInterrupt:
            print('Ctrl-C detected, Just Quitting')
            break

        except Exception as err:
            print(traceback.print_exc())
            print("Unexpected error: ", err)
            break

    if quit_event.is_set():
        await actuator.command_actuator_velocity(des_velocity=0)
        actuator.update_camController_gains(kp_gain=15 ,kd_gain=0.1)
        actuator_controller.update_controller_variables(setpoint_type=SetpointType.HOME_POSITION, setpoint_value=3)
        while True:
            await actuator.read_data()
            home_point_reached = await actuator_controller.command()
            if(home_point_reached): break
            print("Commanding home position...")
            await asyncio.sleep(target_period)
        print("I'm Home, Now Exiting Gracefully")
        print(error_tracking)
        quit_event.clear()

    await actuator.close()
    if daq_task is not None:
        daq_task.close()
    print('Done')

if __name__ == "__main__":
    try:
        # Initialize DAQ instead of Phidget
        initialize_daq()
        
        # Run the main async function
        asyncio.run(main())

    except KeyboardInterrupt:
        print("Program interrupted.")
        if daq_task is not None:
            daq_task.close()

    except Exception as e:
        print(f"Unexpected error: {e}")
        if daq_task is not None:
            daq_task.close()