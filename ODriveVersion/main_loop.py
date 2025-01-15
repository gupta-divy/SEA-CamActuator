import ACTUATOR_CODE_ODRIVE
import time
import traceback
import Controllers
from Controllers import SetpointType
import threading
from keyboard_interrupt_parser import ParameterParser

# Main Code
# datafile_name = input('Input the Filename, if not given data wont be logged: ')
datafile_name = "test0"
actuator = ACTUATOR_CODE_ODRIVE.connect_to_actuator(dataFile_name=datafile_name)
actuator.initial_calibration()
print('Start!')

# Setup controller
actuator_controller = Controllers.CommandSetpoint(actuator)

# Defining controller variables
target_period = 1/actuator.config.control_loop_freq
t0 = time.perf_counter()
last_actuation_time = t0

# keyboard interrupt initialization
lock = threading.Lock()
quit_event = threading.Event()
new_setpoint_event = threading.Event()

keyboard_thread = ParameterParser(lock=lock, quit_event=quit_event, new_setpoint_event=new_setpoint_event)

while True:
    try:
        while time.perf_counter()-last_actuation_time<target_period:
            pass
        time_now = time.perf_counter()

        lock.acquire()
        if new_setpoint_event.is_set():
            actuator_controller.update_controller_variables(setpoint_type=keyboard_thread.setpoint_type,
                                                            setpoint_value=keyboard_thread.setpoint_val)
            new_setpoint_event.clear()
        if quit_event.is_set():
            break
        lock.release()

        last_actuation_time = time_now
        loop_time = time_now - t0
        actuator.read_data(loop_time=loop_time)
        # actuator_controller.command()
        actuator.write_data()

    except KeyboardInterrupt:
        print('Ctrl-C detected, Getting Actuator to Home Position')
        actuator_controller.update_controller_variables(setpoint_type=SetpointType.HOME_POSITION, setpoint_value=0)
        while(not actuator_controller.command()): time.sleep(target_period)
        print("I'm Home, Now Existing Gracefully")
        break

    except Exception as err:
        print(traceback.print_exc())
        print("Unexpected error: ", err)
        break

if quit_event.is_set():
    actuator_controller.update_controller_variables(setpoint_type=SetpointType.HOME_POSITION, setpoint_value=0)
    while(not actuator_controller.command()): time.sleep(target_period)
    print("I'm Home, Now Existing Gracefully")
    quit_event.clear()

actuator.close()
print('Done')
