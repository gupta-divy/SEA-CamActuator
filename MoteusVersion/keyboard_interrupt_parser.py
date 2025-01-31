import threading
from typing import Type
from Controllers import SetpointType

class ParameterParser(threading.Thread):
    def __init__(self, lock: Type[threading.Lock], 
                 new_setpoint_event: Type[threading.Event], 
                 quit_event: Type[threading.Event],
                 gain_event: Type[threading.Event], 
                 name='keyboard-input-thread'):
        super().__init__(name=name, group=None)
        self.setpoint_type: SetpointType = SetpointType.NONE
        self.setpoint_val: float = 0.0
        self.control_gain: float = 0.0
        self.control_gain_type: str = ''
        self.setpoint_event = new_setpoint_event
        self.quit_event = quit_event
        self.gain_event = gain_event
        self.lock = lock
        self.daemon = True  # Ensures thread exits when the main program ends
        self.start()

    def run(self):
        print(
            "Input options: \n"
            "a: actuator_angle, v: actuator_velocity, t: actuator_torque, \n"
            "c: cam_angle, f: cable_force, h: home_position, q: quit\n"
        )
        while not self.quit_event.is_set():
            try:
                msg = input("Enter command: ").strip()
                if not msg:
                    continue

                f_word = msg[0].lower()
                content = msg[1:].strip()

                if f_word == 'q':
                    with self.lock:
                        self.quit_event.set()
                    print("Quit command received. Exiting input thread.")
                    break
                
                try:
                    value = float(content) if content else 0.0
                except ValueError:
                    print("Invalid value. Please enter a valid number after the command letter.")
                    continue


                if f_word == 'p' or f_word == 'd':
                    with self.lock:
                        if f_word=='p':
                            self.control_gain_type = 'p'
                            self.control_gain = value
                        elif f_word=='d':
                            self.control_gain_type = 'd'
                            self.control_gain = value
                        else:
                            print('Invalid input')
                            continue
                            
                        self.gain_event.set()
                        print("Control Gains Updated: ", self.control_gain_type, ' Value: ', self.control_gain)
                        continue

                with self.lock:
                    if f_word == 'a':
                        self.setpoint_type = SetpointType.ACTUATOR_ANGLE
                        self.setpoint_val = value
                    elif f_word == 'v':
                        self.setpoint_type = SetpointType.ACTUATOR_VELOCITY
                        self.setpoint_val = value
                    elif f_word == 't':
                        self.setpoint_type = SetpointType.ACTUATOR_TORQUE
                        self.setpoint_val = value
                    elif f_word == 'c':
                        self.setpoint_type = SetpointType.CAM_ANGLE
                        self.setpoint_val = value
                    elif f_word == 'f':
                        self.setpoint_type = SetpointType.CABLE_FORCE
                        self.setpoint_val = value
                    elif f_word == 'h':
                        self.setpoint_type = SetpointType.HOME_POSITION
                        self.setpoint_val = 0.0
                    else:
                        print("Invalid command. Please try again.")
                        continue

                    self.setpoint_event.set()
                    print(f"Setpoint updated: Type={self.setpoint_type}, Value={self.setpoint_val}")

            except EOFError:
                print("Input stream closed. Exiting input thread.")
                self.quit_event.set()
                break

        print("Input thread terminated.")
