import ACTUATOR_CODE_MOTEUS
import time
import traceback
import Controllers
import odrive
from Controllers import SetpointType
import threading
from keyboard_interrupt_parser import ParameterParser
import asyncio
import math
import numpy as np
from Phidget22.Phidget import *
from Phidget22.Devices.VoltageRatioInput import *
from scipy.signal import chirp

FILENAME = "dist_test_amp20"

# Load Cell Calibration Parameters (determined through calibration)
CALIBRATION_FACTOR = -4543496.53309774
TARE_VALUE = None
measured_force = None
def onVoltageRatioChange(self, voltageRatio):
    global TARE_VALUE
    global measured_force
    # Capture the first reading as the TARE value
    if TARE_VALUE is None:
        TARE_VALUE = voltageRatio
        print(f"TARE Value Set: {TARE_VALUE:.6f}")
    adjusted_ratio = voltageRatio - TARE_VALUE
    measured_force = adjusted_ratio * CALIBRATION_FACTOR * 9.81 / 1000

#Odrive - Disturbance Injector
def connect_odrive():
    print("Connecting to ODrive...")
    try:
        odrv0 = odrive.find_any()
        print("ODrive connected!")
        return odrv0
    except Exception as e:
        print(f"Failed to connect to ODrive: {e}")
        return None

def setup_motor_for_velocity_control(odrv0):
    odrv0.axis0.requested_state = 8                 # Closed-loop control
    odrv0.axis0.controller.config.control_mode = 2  # Velocity control mode
    odrv0.axis0.controller.config.input_mode = 2    # Input mode for velocity control
    odrv0.axis0.controller.config.vel_ramp_rate = 2000
    odrv0.axis0.controller.config.torque_ramp_rate = 0.2

def get_dist_velocity_vector(sample_rate=200,duration=10,start_vel=1,end_vel=10):
    velocity_vector = np.linspace(start=start_vel, stop=end_vel, num=sample_rate * duration)
    return velocity_vector

# Main Moteus Controller Code
async def main(odrv0, voltageRatioInput):
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
    
    #Initialization Parameters
    #Input Signal:
    set_signal = 20
    duration = 15
    sample_rate = actuator.config.control_loop_freq
    amplitude = 15
    frequency_start = 0
    frequency_end = 10

    ## Bandwidth Test
    # t = np.linspace(0, duration, int(sample_rate * duration))
    # signal = amplitude * chirp(t, f0=frequency_start, f1=frequency_end, t1=duration, method='linear', phi=-90) 
    
    ## Disturbance Test
    velocity_vector = get_dist_velocity_vector(sample_rate=sample_rate,duration=duration,start_vel=frequency_start,end_vel=frequency_end)
    pos_offset = odrv0.axis0.pos_estimate

    t0 = time.perf_counter()
    last_actuation_time = t0
    last_print_time = t0
    actuator_controller.update_controller_variables(setpoint_type=SetpointType.CAM_ANGLE, setpoint_value=set_signal)
    i = 0
    out_bound_count = 0
    updated = False

    while True:
        try:
            while time.perf_counter() - last_actuation_time < target_period:
                pass

            time_now = time.perf_counter()
            
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
            
            # ## Bandwidth Testing
            # if loop_time>5 and i<len(signal):
            #     # print(set_signal+signal[i])
            #     actuator_controller.update_controller_variables(setpoint_type=SetpointType.CAM_ANGLE, setpoint_value=set_signal+signal[i])
            #     i+=1
            # if i==len(signal):
            #     "Done with Controller Testing"
            #     quit_event.set()
            #     break

            # ## Disturbance Testing
            if loop_time>5 and i<len(velocity_vector):
                odrv0.axis0.controller.input_vel = velocity_vector[i]
                vel = odrv0.axis0.vel_estimate
                pos = odrv0.axis0.pos_estimate
                actuator.data.disturbance_velocity_measured = vel
                actuator.data.disturbance_displacement = pos - pos_offset
                i += 1
            if i==len(velocity_vector):
                "Done with Controller Testing"
                quit_event.set()
                break
            
            actuator.data.measured_force = measured_force
            error_tracking += abs(actuator_controller.setpoint_value - actuator.data.cam_angle) if actuator_controller.setpoint_type==SetpointType.CAM_ANGLE else 0
            await actuator_controller.command()
            actuator.write_data()
            if actuator_controller.setpoint_type == SetpointType.CAM_ANGLE and (actuator.data.cam_angle>42 or actuator.data.cam_angle<2):
                out_bound_count += 1
            if out_bound_count>=15: 
                print("Quitting Velocity: ", velocity_vector[i], vel)
                quit_event.set()
                break

        except KeyboardInterrupt:
            odrv0.axis0.controller.input_vel = 0  # Stop movement
            odrv0.axis0.requested_state = 1
            print('Ctrl-C detected, Just Quitting')
            break

        except Exception as err:
            odrv0.axis0.controller.input_vel = 0  # Stop movement
            odrv0.axis0.requested_state = 1
            print(traceback.print_exc())
            print("Unexpected error: ", err)
            break

    if quit_event.is_set():
        odrv0.axis0.controller.input_vel = 0  # Stop movement
        odrv0.axis0.requested_state = 1
        await actuator.command_actuator_velocity(des_velocity=0)
        actuator.update_camController_gains(kp_gain=50 ,kd_gain=0.2)
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

    odrv0.axis0.requested_state = 1  # Switch to idle mode
    print("ODrive set to idle mode. Exiting.")
    await actuator.close()
    voltageRatioInput.close()
    print('Done')

if __name__ == "__main__":
    try:
        voltageRatioInput = VoltageRatioInput()
        odrv0 = connect_odrive()
        if odrv0 is None:
            exit()
        setup_motor_for_velocity_control(odrv0)
        voltageRatioInput.setOnVoltageRatioChangeHandler(onVoltageRatioChange)
        # Open Phidget and wait for attachment
        voltageRatioInput.openWaitForAttachment(5000)
        voltageRatioInput.setDataInterval(25)
        asyncio.run(main(odrv0=odrv0, voltageRatioInput=voltageRatioInput))

    except KeyboardInterrupt:
        odrv0.axis0.controller.input_vel = 0
        odrv0.axis0.requested_state = 1
        print("Program interrupted.")

    except Exception as e:
        print(f"Unexpected error: {e}")
