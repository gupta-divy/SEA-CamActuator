#!/usr/bin/env python3
import ACTUATOR_CODE
import time
import traceback
import csv  # Added import
import numpy as np
from Controllers import SpringMechController
import math

# Configuration
control_freq = 200

# Main_Code
datafile_name = input('Input the Filename, if not given data wont be logged: ')
actuator = ACTUATOR_CODE.connect_to_actuator(dataFile_name=datafile_name)
actuator.initial_calibration()  
print('Start!')

CAM_KP = 0.6
CAM_KI = 0
CAM_KD = 0.020
SAT_VALUE = 20

subfolder_name = 'exo_data/'
filename = subfolder_name + time.strftime("%Y%m%d_%H%M_") + datafile_name + '.csv'

target_period = 1 / control_freq
cam_angles = np.arange(5, 70, 5) 
cam_angles = [cam_angles,np.arange(70, 5, 5)]
interrupt = False
curr_lim = False

for angle in cam_angles:
    # Controller Initialized
    cam_controller = SpringMechController(
        mechanism=actuator,
        desired_cam_ang=angle,
        cam_Kp=CAM_KP,
        cam_Ki=CAM_KI,
        cam_Kd=CAM_KD,
        Kp=actuator.constants.default_KP,
        Kd=actuator.constants.default_KD,
        Ki=actuator.constants.default_KI,
        ff=actuator.constants.default_FF,
        saturation_value=SAT_VALUE
    )
    t0 = time.perf_counter()
    last_actuation_time = t0
    # Moved inside the loop after initialization
    error_queue = []
    print("Read load cell value")
    time.sleep(5)
    print("set_cam_angle: ", angle)
    while True:
        try:
            elapsed = time.perf_counter() - last_actuation_time
            if elapsed < target_period:
                time.sleep(target_period - elapsed)
            time_now = time.perf_counter()
            last_actuation_time = time_now
            loop_time = time_now - t0
            actuator.read_data(loop_time=loop_time)
            cam_controller.command()
            error = actuator.data.ang_error
            actuator.write_data()
            if actuator.data.motor_current > 2000:
                print("Current limit reached ")
                curr_lim = True
                break
            
            if len(error_queue)<=100:
                error_queue.append(error)
            else:
                error_queue.pop(0)
                error_queue.append(abs(error))
                if (sum(error_queue)/len(error_queue))<1:
                    break
                if loop_time>3:
                    break
            
        except KeyboardInterrupt:
            print('Ctrl-C detected, Exiting Gracefully')
            interrupt = True
            break

        except Exception as err:
            print(traceback.format_exc())
            print("Unexpected error: ", err)
            break

    if curr_lim:
        break

    if interrupt:
        break

actuator.close()
print('Done')
