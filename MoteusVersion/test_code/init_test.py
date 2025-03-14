import asyncio
import moteus
from moteus import multiplex as moteusMp
import time
import math
import threading

# Shared flag to stop the loop
stop_flag = False
motor_position = 0.5

# Thread function to listen for "e" input
def input_listener(c):
    global stop_flag, motor_position
    while not stop_flag:
        user_input = input().strip().lower()
        if user_input == "e":
            stop_flag = True
            print("Stopping the motor and exiting...")
        elif user_input.startswith("p") and len(user_input) > 1:
            try:
                motor_position = float(user_input[1:])
                print(f"Updated motor position to: {motor_position}")
            except ValueError:
                print("Invalid position input. Please use 'p<number>' format.")

async def main():
    global stop_flag
    global motor_position

    # Communicate with controller ID 1 on the default transport
    c = moteus.Controller()

    # Start the input listener thread
    input_thread = threading.Thread(target=input_listener, args=(c,))
    input_thread.daemon = True  # Ensures the thread exits when the main program exits
    input_thread.start()

    # Prepare the controller
    await c.set_stop()
    to_query = {
                moteus.Register.MODE: moteusMp.INT8,
                moteus.Register.POSITION: moteusMp.F32,
                moteus.Register.VELOCITY: moteusMp.F32,
                moteus.Register.ENCODER_2_POSITION: moteusMp.F32,
                moteus.Register.ENCODER_2_VELOCITY: moteusMp.F32,
                moteus.Register.TORQUE: moteusMp.F32,
                moteus.Register.FAULT: moteusMp.INT8,
                moteus.Register.TEMPERATURE: moteusMp.INT8,
                moteus.Register.VOLTAGE: moteusMp.INT8,
                moteus.Register.POWER: moteusMp.INT32,
                moteus.Register.MILLISECOND_COUNTER: moteusMp.INT16,
                moteus.Register.Q_CURRENT: moteusMp.F32,
                moteus.Register.VFOC_VOLTAGE: moteusMp.F32,
                moteus.Register.COMMAND_VELOCITY: moteus.F32,
                moteus.Register.COMMAND_VELOCITY_LIMIT: moteus.INT16 
            }

    while not stop_flag:
        try:
            # Send position command
            t0 = time.time()
            # await c.set_position(position=motor_position, query=False)
            t1 = time.time()

            # Query controller data
            data_feed = await c.custom_query(to_query)
            t2 = time.time()

            # Debug information
            print("Set Time: ", t1 - t0)
            print("Query Time: ", t2 - t1)
            print(data_feed)
            print(data_feed.values)


            # Non-blocking sleep
            await asyncio.sleep(0.1) 

        except Exception as e:
            print(f"An error occurred: {e}")
            break

    # Ensure motor is stopped before exiting
    await c.set_stop()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Program interrupted.")
