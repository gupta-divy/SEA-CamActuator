import ACTUATOR_CODE_MOTEUS
import time
import traceback
import Controllers
import odrive
import asyncio
import threading
from Controllers import SetpointType
from simple_performance_monitor import SimplePerformanceMonitor 

FILENAME = "distTest_amp25"
POS_OFFSET = 0
SET_ODRIVE_VEL = 0
RUNNING = True  # Control flag for both threads

# Shared dictionary to hold ODrive state
odrive_state = {
    "pos_estimate": 0,
    "vel_estimate": 0
}

# ODrive - Disturbance Injector
def connect_odrive():
    global POS_OFFSET
    print("Connecting to ODrive...")
    try:
        odrv0 = odrive.find_any()
        print("ODrive connected!")
        POS_OFFSET = odrv0.axis0.pos_estimate
        return odrv0
    except Exception as e:
        print(f"Failed to connect to ODrive: {e}")
        return None

def setup_motor_for_velocity_control(odrv0):
    odrv0.axis0.requested_state = 8                 # Closed-loop control
    odrv0.axis0.controller.config.control_mode = 2  # Velocity control mode
    odrv0.axis0.controller.config.input_mode = 2    # Input mode for velocity control
    odrv0.axis0.controller.config.vel_ramp_rate = 2000
    odrv0.axis0.controller.config.torque_ramp_rate = 0.2

def get_dist_velocity(t, start_vel=1, end_vel=10, duration=15):
    vel = start_vel + ((end_vel - start_vel) / duration) * t
    return vel if t < duration else None 

# ODrive Control Loop in a Separate Thread
def odrive_control_loop(odrv0):
    global RUNNING, odrive_state
    try:
        while RUNNING:  # Check global flag
            odrv0.axis0.controller.input_vel = SET_ODRIVE_VEL
            odrive_state["pos_estimate"] = odrv0.axis0.pos_estimate
            odrive_state["vel_estimate"] = odrv0.axis0.vel_estimate
            time.sleep(0.005)  # Poll ODrive data every 10ms
    except Exception as e:
        print(f"ODrive Thread Error: {e}")
    finally:
        # Ensure ODrive stops when exiting
        odrv0.axis0.controller.input_vel = 0
        odrv0.axis0.requested_state = 1
        print("ODrive set to idle mode. Exiting ODrive thread.")

# Main Moteus Controller Code
async def main():
    global SET_ODRIVE_VEL, RUNNING
    actuator = await ACTUATOR_CODE_MOTEUS.connect_to_actuator(dataFile_name=FILENAME)
    await actuator.initial_calibration()
    print('Start!')

    actuator_controller = Controllers.CommandSetpoint(actuator)
    actuator_controller.update_controller_variables(setpoint_type=SetpointType.CAM_ANGLE, setpoint_value=20)

    target_period = 1 / actuator.config.control_loop_freq
    perf_monitor = SimplePerformanceMonitor()
    perf_monitor.start(target_period=target_period)
    t0 = time.perf_counter()
    last_actuation_time = t0
    out_bound_count = 0
    vel = 0
    quit_vel = None

    while True:  # Check global flag
        try:
            while time.perf_counter() - last_actuation_time < target_period:
                pass
            perf_monitor.record_cycle()
            time_now = time.perf_counter()
            last_actuation_time = time_now
            loop_time = time_now - t0
            await actuator.read_data(loop_time=loop_time)
            await actuator_controller.command()

            # Disturbance Testing
            if loop_time > 5:
                vel = get_dist_velocity(loop_time - 5)
                SET_ODRIVE_VEL = vel if vel is not None else 0
                # Read ODrive values from shared dictionary
                actuator.data.measured_disturbance_velocity = odrive_state["vel_estimate"]
                actuator.data.measured_disturbance_displacement = odrive_state["pos_estimate"] - POS_OFFSET
            actuator.write_data()

            if vel is None:
                RUNNING = False
                break
            if actuator_controller.setpoint_type == SetpointType.CAM_ANGLE and (actuator.data.cam_angle > 35 or actuator.data.cam_angle < 5):
                out_bound_count += 1
            if out_bound_count > 5: 
                RUNNING = False  # Stop ODrive thread
                if quit_vel is None: 
                    quit_vel = get_dist_velocity(loop_time - 5)
                    print("Quitting Velocity: ", quit_vel)
                if actuator.data.measured_disturbance_velocity<0.05:
                    break

        except KeyboardInterrupt:
            print('Ctrl-C detected, Just Quitting')
            break

        except Exception as err:
            print(traceback.print_exc())
            print("Unexpected error: ", err)
            break

    RUNNING = False
    await actuator.close()
    print('Moteus closed properly, printing control loop stats')
    perf_monitor.print_stats()
    
if __name__ == "__main__":
    try:
        odrv0 = connect_odrive()
        if odrv0 is None:
            exit()
        setup_motor_for_velocity_control(odrv0)
        odrive_thread = threading.Thread(target=odrive_control_loop, args=(odrv0,), daemon=True)
        odrive_thread.start()
        asyncio.run(main())
        odrive_thread.join()

    except KeyboardInterrupt:
        RUNNING = False 
        print("Program interrupted.")

    except Exception as e:
        RUNNING = False 
        print(f"Unexpected error: {e}")
