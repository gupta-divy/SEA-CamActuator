import ACTUATOR_CODE
import time
import traceback
import numpy as np
from Controllers import SpringMechController

# Configuration
control_freq = 200

# Main_Code
datafile_name = input('Input the Filename, if not given data wont be logged: ')
actuator = ACTUATOR_CODE.connect_to_actuator(dataFile_name=datafile_name)
actuator.intial_calibration()
desired_angle = int(input('Enter the cam_angle you want the controller to track: '))
print('Start!')

CAM_KP = 0.6
CAM_KI = 0
CAM_KD = 0.020
SAT_VALUE = 20

#Controller Intiialized
cam_controller = SpringMechController(mechanism=actuator,desired_cam_ang=desired_angle,
                                      cam_Kp=CAM_KP, cam_Ki=CAM_KI, cam_Kd=CAM_KD,
                                      Kp=actuator.constants.default_KP, Kd=actuator.constants.default_KD,
                                      Ki=actuator.constants.default_KI, ff=actuator.constants.default_FF,
                                      saturation_value=SAT_VALUE)

# Defining variables
target_period = 1/control_freq
t0 = time.perf_counter()
last_actuation_time = t0

# Controller Commanded
cam_controller.command(reset=True)
while True:
    try:
        while time.perf_counter()-last_actuation_time<target_period:
            pass
        time_now = time.perf_counter()
        last_actuation_time = time_now
        loop_time = time_now - t0
        actuator.read_data(loop_time=loop_time)
        error = cam_controller.command()
        actuator.write_data()

    except KeyboardInterrupt:
        print('Ctrl-C detected, Exiting Gracefully')
        break

    except Exception as err:
        print(traceback.print_exc())
        print("Unexpected error: ", err)
        break

actuator.close()
print('Done')
