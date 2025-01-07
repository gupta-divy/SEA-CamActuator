import ACTUATOR_CODE_ODRIVE
import time
import traceback
from Controllers import CommandSetpoint, SetpointType

# Main Code
datafile_name = input('Input the Filename, if not given data wont be logged: ')
actuator = ACTUATOR_CODE_ODRIVE.connect_to_actuator(dataFile_name=datafile_name)
actuator.intial_calibration()
desired_angle = int(input('Enter the cam_angle you want the controller to track: '))
print('Start!')

# Setup controller
setpoint_type = SetpointType.NONE
setpoint_val = None
actuator_controller = CommandSetpoint(actuator=actuator,setpoint_type=setpoint_type, setpoint_value=setpoint_val)

# Defining controller variables
target_period = 1/actuator.config.control_loop_freq
t0 = time.perf_counter()
last_actuation_time = t0

while True:
    try:
        while time.perf_counter()-last_actuation_time<target_period:
            pass
        time_now = time.perf_counter()
        last_actuation_time = time_now
        loop_time = time_now - t0
        actuator.read_data(loop_time=loop_time)
        actuator_controller.command()
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

actuator.close()
print('Done')
