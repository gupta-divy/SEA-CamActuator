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
def input_listener():
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

async def query_with_timeout(controller, to_query, timeout=0.5):
    """Query controller with a timeout to prevent blocking indefinitely"""
    try:
        # Create a future for the query
        query_task = asyncio.create_task(controller.custom_query(to_query))
        # Wait for the task to complete with a timeout
        data = await asyncio.wait_for(query_task, timeout)
        return data
    except asyncio.TimeoutError:
        print(f"Query to controller {controller.id} timed out after {timeout}s")
        return None
    except Exception as e:
        print(f"Error querying controller {controller.id}: {e}")
        return None

async def main():
    global stop_flag
    global motor_position

    # For the first controller
    transport1 = moteus.Fdcanusb(path="COM3")  # Change if needed
    
    # For the second controller
    transport2 = moteus.Fdcanusb(path="COM4")  # Change if needed
    
    # Create controllers with specific IDs and transports
    c1 = moteus.Controller(id=1, transport=transport1)
    c2 = moteus.Controller(id=2, transport=transport2)
    
    print(f"Controller 1 (COM3): {c1}")
    print(f"Controller 2 (COM4): {c2}")

    # Start the input listener thread
    input_thread = threading.Thread(target=input_listener)
    input_thread.daemon = True
    input_thread.start()

    # Define what to query - just position for efficiency
    to_query = {
        moteus.Register.POSITION: moteusMp.F32,
        moteus.Register.FAULT: moteusMp.INT8
    }

    # Prepare the controllers with timeout
    try:
        await asyncio.wait_for(c1.set_stop(), 1.0)
        print("Controller 1 stopped successfully")
    except asyncio.TimeoutError:
        print("Stopping controller 1 timed out")
    except Exception as e:
        print(f"Error stopping controller 1: {e}")
    
    try:
        await asyncio.wait_for(c2.set_stop(), 1.0)
        print("Controller 2 stopped successfully")
    except asyncio.TimeoutError:
        print("Stopping controller 2 timed out")
    except Exception as e:
        print(f"Error stopping controller 2: {e}")

    print("\n=== Starting main loop ===\n")
    
    while not stop_flag:
        # Query controller 1 with timeout
        data_c1 = await query_with_timeout(c1, to_query)
        if data_c1:
            pos1 = data_c1.values.get(moteus.Register.POSITION, "N/A")
            fault1 = data_c1.values.get(moteus.Register.FAULT, "N/A")
            print(f"Controller 1 (COM3): Position={pos1}, Fault={fault1}")
        
        # Query controller 2 with timeout
        data_c2 = await query_with_timeout(c2, to_query)
        if data_c2:
            pos2 = data_c2.values.get(moteus.Register.POSITION, "N/A")
            fault2 = data_c2.values.get(moteus.Register.FAULT, "N/A")
            print(f"Controller 2 (COM4): Position={pos2}, Fault={fault2}")
        else:
            print("Controller 2 (COM4): No response")
        
        # Non-blocking sleep
        await asyncio.sleep(0.1)

    # Ensure motors are stopped before exiting (with timeouts)
    try:
        await asyncio.wait_for(c1.set_stop(), 1.0)
    except (asyncio.TimeoutError, Exception) as e:
        print(f"Error stopping controller 1: {e}")
    
    try:
        await asyncio.wait_for(c2.set_stop(), 1.0)
    except (asyncio.TimeoutError, Exception) as e:
        print(f"Error stopping controller 2: {e}")
    
    print("Both controllers stopped")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Program interrupted.")
    except Exception as e:
        print(f"Unhandled exception: {e}")
        import traceback
        traceback.print_exc()