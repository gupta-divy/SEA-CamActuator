import ACTUATOR_CODE_MOTEUS
import time
import traceback
import Controllers

from Controllers import SetpointType
import threading
from keyboard_interrupt_parser import ParameterParser
import asyncio
import math

async def main():
    datafile_name = "test0"
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

            if time_now - last_print_time >= 0.5:
                # print("Velocity: ", actuator.data.actuator_velocity, "Cam Angle: ", actuator.data.cam_angle, "Gains", actuator.config.camControllerGainKp, actuator.config.camControllerGainKd)
                # dist_compensation = (actuator.data.disturbance_velocity / (1000*actuator.design_constants.ACTUATOR_RADIUS)) * (180 / math.pi)
                print("Torque: ", actuator.data.actuator_torque, "Commanded: ", actuator.data.commanded_actuator_torque)
                last_print_time = time_now  # Update the last controller update timestamp

            await actuator.read_data(loop_time=loop_time)
            # error_tracking += abs(actuator_controller.setpoint_value - actuator.data.cam_angle) if actuator_controller.setpoint_type==SetpointType.CAM_ANGLE else 0
            await actuator_controller.command()
            actuator.write_data()
            if actuator_controller.setpoint_type == SetpointType.CAM_ANGLE and (actuator.data.cam_angle>71 or actuator.data.cam_angle<2): 
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
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Program interrupted.")
    except Exception as e:
        print(f"Unexpected error: {e}")
