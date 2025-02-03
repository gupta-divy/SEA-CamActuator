import ACTUATOR_CODE_MOTEUS
import time
import traceback
import Controllers
from Controllers import SetpointType
import threading
from keyboard_interrupt_parser import ParameterParser
import asyncio
import numpy as np

async def main():
    datafile_name = "test0"
    actuator = await ACTUATOR_CODE_MOTEUS.connect_to_actuator(dataFile_name=datafile_name)
    # input("Press Enter to Start")
    await actuator.initial_calibration()
    print('Start!')

    # Controller Tuning
    error_tracking = 0
    Kp = np.linspace(start=100,stop=160,num=8)
    Kd = np.linspace(start=0,stop=0.25,num=5)
    kd, kp = np.meshgrid(Kd, Kp)
    gain_combinations = np.column_stack((kp.ravel(), kd.ravel()))
    gain_in_test = 0
    error_dict = {}
    steady_state_err = []

    # Setup controller
    actuator_controller = Controllers.CommandSetpoint(actuator)
    actuator_controller.update_controller_variables(setpoint_type=SetpointType.CAM_ANGLE,setpoint_value=30)
    actuator.update_camController_gains(kp_gain=gain_combinations[gain_in_test][0], kd_gain=gain_combinations[gain_in_test][1])

    # Keyboard interrupt initialization
    lock = threading.Lock()
    quit_event = threading.Event()
    new_setpoint_event = threading.Event()
    gain_event = threading.Event()
    keyboard_thread = ParameterParser(lock=lock, quit_event=quit_event, new_setpoint_event=new_setpoint_event, gain_event=gain_event)

    # Defining controller variables
    target_period = 1 / actuator.config.control_loop_freq
    t0 = time.perf_counter()
    last_actuation_time = t0
    last_print_time = t0
    last_gain_update = t0

    while True:
        try:
            while time.perf_counter() - last_actuation_time < target_period:
                await asyncio.sleep(0.001)  # Non-blocking wait
            time_now = time.perf_counter()
            last_actuation_time = time_now
            loop_time = time_now - t0 

            # Keyboard Input Watcher
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
            
            # Controller Tuning
            steady_state_err.append(abs(actuator.data.cam_angle_error))
            if len(steady_state_err) > int(0.5*actuator.config.control_loop_freq) : steady_state_err.pop(0)

            if time_now - last_gain_update>10 and np.average(steady_state_err)<0.5:
                error_dict[gain_in_test] = error_tracking
                gain_in_test+=1
                if gain_in_test==len(gain_combinations): break
                actuator.update_camController_gains(kp_gain=gain_combinations[gain_in_test][0],
                                                    kd_gain=gain_combinations[gain_in_test][1])
                last_gain_update = time_now
                error_tracking = 0
                print("Velocity: ", actuator.data.actuator_velocity, "Cam Angle: ", actuator.data.cam_angle, "Gains", actuator.config.camControllerGainKp, actuator.config.camControllerGainKd)

            if loop_time>2 :
                error_tracking += abs(actuator.data.cam_angle_error)

            # Main commands
            await actuator.read_data(loop_time=loop_time)
            await actuator_controller.command()
            actuator.write_data()

            # if time_now - last_print_time >= 0.1:
                # print("Velocity: ", actuator.data.actuator_velocity, "Cam Angle: ", actuator.data.cam_angle, "Gains", actuator.config.camControllerGainKp, actuator.config.camControllerGainKd)
                # last_print_time = time_now  # Update the last controller update timestamp
            
            # Safety Measure for unstable gain values
            if actuator_controller.setpoint_type == SetpointType.CAM_ANGLE and (actuator.data.cam_angle>60 or actuator.data.cam_angle<3): 
                await actuator.command_actuator_velocity(des_velocity=0)
                break

        except KeyboardInterrupt:
            print('Ctrl-C detected, Getting Actuator to Home Position')
            break

        except Exception as err:
            traceback.print_exc()
            print("Unexpected error: ", err)
            break

    if quit_event.is_set():
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
    print(error_dict)
    print('Done')

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Program interrupted.")
    except Exception as e:
        print(f"Unexpected error: {e}")
