import ACTUATOR_CODE_MOTEUS
import time
import traceback
import Controllers
from Controllers import SetpointType
import threading
from keyboard_interrupt_parser import ParameterParser
import asyncio

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

    # Keyboard interrupt initialization
    lock = threading.Lock()
    quit_event = threading.Event()
    new_setpoint_event = threading.Event()

    keyboard_thread = ParameterParser(lock=lock, quit_event=quit_event, new_setpoint_event=new_setpoint_event)

    while True:
        try:
            while time.perf_counter() - last_actuation_time < target_period:
                await asyncio.sleep(0.001)  # Non-blocking wait

            time_now = time.perf_counter()

            with lock:
                if new_setpoint_event.is_set():
                    print(f"Updating controller: type={keyboard_thread.setpoint_type}, value={keyboard_thread.setpoint_val}")
                    actuator_controller.update_controller_variables(
                        setpoint_type=keyboard_thread.setpoint_type,
                        setpoint_value=keyboard_thread.setpoint_val,
                    )
                    new_setpoint_event.clear()
                if quit_event.is_set():
                    break

            last_actuation_time = time_now
            loop_time = time_now - t0 

            await actuator.read_data(loop_time=loop_time)
            # print(actuator.data.cam_angle, actuator.data.cam_encoder_raw)
            await actuator_controller.command()
            actuator.write_data()

        except KeyboardInterrupt:
            print('Ctrl-C detected, Getting Actuator to Home Position')
            
            break

        except Exception as err:
            print(traceback.print_exc())
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
