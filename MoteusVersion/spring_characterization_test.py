import ACTUATOR_CODE_MOTEUS
import time
import traceback
import Controllers
from Controllers import SetpointType
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
    last_controller_update_time = t0
    direction = 1
    while True:
        try:
            while time.perf_counter() - last_actuation_time < target_period:
                await asyncio.sleep(0.001)  # Non-blocking wait

            time_now = time.perf_counter()
            last_actuation_time = time_now
            loop_time = time_now - t0

            if(direction==1 and actuator.data.cam_angle<=65.0):
                value = actuator.data.cam_angle + 5
            else: direction = 0

            if(direction==0 and actuator.data.cam_angle>=5):
                value = actuator.data.cam_angle - 5
            
            if(actuator.data.cam_angle<=5.0):
                value = actuator.data.cam_angle
                break

            if time_now - last_controller_update_time >= 10:
                actuator_controller.update_controller_variables(setpoint_type=SetpointType.CAM_ANGLE, setpoint_value=value)
                last_controller_update_time = time_now  # Update the last controller update timestamp

            await actuator.read_data(loop_time=loop_time)
            await actuator_controller.command()
            actuator.write_data()

            if actuator_controller.setpoint_type == SetpointType.CAM_ANGLE and (actuator.data.cam_angle>68 or actuator.data.cam_angle<4): 
                await actuator.command_actuator_velocity(des_velocity=0) 
                break

        except KeyboardInterrupt:
            print('Ctrl-C detected, Getting Actuator to Home Position')
            break

        except Exception as err:
            print(traceback.print_exc())
            print("Unexpected error: ", err)
            break

    await actuator.close()
    print('Done')

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Program interrupted.")
    except Exception as e:
        print(f"Unexpected error: {e}")
