import ACTUATOR_CODE_MOTEUS
import time
import traceback
import Controllers
from Controllers import SetpointType
import asyncio
import math
import numpy as np

def generate_test_frequencies():
    """Generate logarithmically spaced frequencies with dense sampling around resonance"""
    # Base parameters
    low_freq = 0.1
    high_freq = 12
    resonance = (7, 11)
    
    # Create logarithmic spacing outside resonance region
    low_range = np.geomspace(low_freq, resonance[0], 10, endpoint=False)
    high_range = np.geomspace(resonance[1], high_freq, 4)
    
    # Linear spacing in resonance region
    res_range = np.linspace(resonance[0], resonance[1], 17)
    
    # Combine and ensure uniqueness
    return np.unique(np.concatenate([low_range, res_range, high_range]))


TEST_FREQ = generate_test_frequencies()

def signal_generator(t, test_freq=TEST_FREQ, dur = 5, ampl=10):
    if((t//dur)%2==1): return 0
    else:
        test_freq_idx = (t//(2*dur)) 
        if((t%dur)>dur-0.01): print("Test frequency:", TEST_FREQ[int(test_freq_idx)])
        use_t = t%dur
        if test_freq_idx>=len(test_freq): return None
        return ampl*math.sin(2*math.pi*TEST_FREQ[int(test_freq_idx)]*use_t)

async def main():
    datafile_name = "bandwidthTest"
    actuator = await ACTUATOR_CODE_MOTEUS.connect_to_actuator(dataFile_name=datafile_name)
    await actuator.initial_calibration()
    print('Start!')

    # Setup controller
    actuator_controller = Controllers.CommandSetpoint(actuator)
    actuator_controller.update_controller_variables(SetpointType.CAM_ANGLE, 20)
    # Defining controller variables
    target_period = 1 / actuator.config.control_loop_freq
    t0 = time.perf_counter()
    last_actuation_time = t0

    while True:
        try:
            while time.perf_counter() - last_actuation_time < target_period:
                pass
            time_now = time.perf_counter()
            last_actuation_time = time_now
            loop_time = time_now - t0 

            await actuator.read_data(loop_time=loop_time)

            if loop_time>5:
                t = loop_time-5
                change = signal_generator(t)
                if change is None:
                    break
                else:
                    cam_angle = 20 + change
                    actuator_controller.update_controller_variables(setpoint_type=SetpointType.CAM_ANGLE, setpoint_value=cam_angle)
            
            await actuator_controller.command()
            actuator.write_data()

            # safety exit
            # if actuator_controller.setpoint_type == SetpointType.CAM_ANGLE and (actuator.data.cam_angle>71 or actuator.data.cam_angle<-3):
            #     print("Going out of bounds") 
            #     await actuator.command_actuator_velocity(des_velocity=0)
            #     break

        except KeyboardInterrupt:
            print('Ctrl-C detected')
            
            break

        except Exception as err:
            print(traceback.print_exc())
            print("Unexpected error: ", err)
            break
    
    print("Exited...")
    await actuator.close()
    print('Done')

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Program interrupted.")
    except Exception as e:
        print(f"Unexpected error: {e}")
