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
    if odrv0 is None:
        print("No ODrive connected, exiting...")
        return
    odrv0.axis0.controller.config.control_mode = 2  # Velocity control mode
    odrv0.axis0.controller.config.input_mode = 2    # Input mode for velocity control
    odrv0.axis0.requested_state = 8  # Closed-loop control
    odrv0.axis0.controller.config.vel_ramp_rate = 2000
    odrv0.axis0.controller.config.torque_ramp_rate = 0.2 # Default is 0.009999
    print("Motor set to velocity control mode and closed-loop control.") 

def get_dist_velocity_vector():
    sample_rate = 200
    duration = 10
    start_vel = 1
    end_vel = 2
    velocity_vector = np.linspace(start=start_vel, stop=end_vel, num=sample_rate * duration)
    return velocity_vector

async def main(odrv0):
    velocity_vector = get_dist_velocity_vector()
    i = 0
    datafile_name = "dist_test0"
    actuator = await ACTUATOR_CODE_MOTEUS.connect_to_actuator(dataFile_name=datafile_name)
    # input("Press Enter to Start")
    await actuator.initial_calibration()
    print('Start!')

    # Setup controller
    actuator_controller = Controllers.CommandSetpoint(actuator)

    # Defining controller variables
    target_period = 1 / actuator.config.control_loop_freq
    t0 = time.perf_counter()
    last_actuation_time = t0
    last_print_time = t0
    # Keyboard interrupt initialization
    lock = threading.Lock() 
    quit_event = threading.Event()
    new_setpoint_event = threading.Event()
    gain_event = threading.Event()
    error_tracking = 0
    keyboard_thread = ParameterParser(lock=lock, quit_event=quit_event, new_setpoint_event=new_setpoint_event, gain_event=gain_event)
    actuator_controller.update_controller_variables(setpoint_type=SetpointType.CAM_ANGLE, setpoint_value=30)
    pos_offset = odrv0.axis0.pos_estimate
    while True:
        try:
            while time.perf_counter() - last_actuation_time < target_period:
                # await asyncio.sleep(0.001)  # Non-blocking wait
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

            if loop_time>10 and i<len(velocity_vector):
                odrv0.axis0.controller.input_vel = velocity_vector[i]
                vel = odrv0.axis0.vel_estimate
                pos = odrv0.axis0.pos_estimate
                actuator.data.disturbance_velocity = vel
                actuator.data.disturbance_displacement = pos - pos_offset
                # print("Commanding_velocity: ", velocity_vector[i], "Actual Velocity: ", vel)
                i += 1

            if i==len(velocity_vector):
                odrv0.axis0.controller.input_vel = 0  # Stop movement
                odrv0.axis0.requested_state = 1  # Switch to idle mode
                print("ODrive set to idle mode. Exiting.")

            # if time_now - last_print_time >= 0.5:
            #     print("Velocity: ", actuator.data.actuator_velocity, "Cam Angle: ", actuator.data.cam_angle, "Gains", actuator.config.camControllerGainKp, actuator.config.camControllerGainKd)
            #     # dist_compensation = (actuator.data.disturbance_velocity / (1000*actuator.design_constants.ACTUATOR_RADIUS)) * (180 / math.pi)
            #     print("Torque: ", actuator.data.actuator_torque, "Commanded: ", actuator.data.commanded_actuator_torque)
            #     last_print_time = time_now  # Update the last controller update timestamp

            await actuator.read_data(loop_time=loop_time)
            # error_tracking += abs(actuator_controller.setpoint_value - actuator.data.cam_angle) if actuator_controller.setpoint_type==SetpointType.CAM_ANGLE else 0
            await actuator_controller.command()
            actuator.write_data()
            if actuator_controller.setpoint_type == SetpointType.CAM_ANGLE and (actuator.data.cam_angle>78 or actuator.data.cam_angle<1): 
                await actuator.command_actuator_velocity(des_velocity=0)
                break

        except KeyboardInterrupt:
            print('Ctrl-C detected, Getting Actuator to Home Position')
            break

        except Exception as err:
            print(traceback.print_exc())
            print("Unexpected error: ", err)
            break

    if quit_event.is_set():
        actuator.update_camController_gains(kp_gain=50 ,kd_gain=0.2)
        actuator_controller.update_controller_variables(setpoint_type=SetpointType.HOME_POSITION, setpoint_value=0)
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
    print('Done')

if __name__ == "__main__":
    try:
        odrv0 = connect_odrive()
        if odrv0 is None:
            exit()
        setup_motor_for_velocity_control(odrv0)
        asyncio.run(main(odrv0=odrv0))

    except KeyboardInterrupt:
        print("Program interrupted.")
    except Exception as e:
        print(f"Unexpected error: {e}")
