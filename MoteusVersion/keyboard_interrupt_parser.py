import threading
from typing import Type
from Controllers import SetpointType

class ParameterParser(threading.Thread):
    def __init__(self, lock: Type[threading.Lock], 
                 new_setpoint_event: Type[threading.Event], 
                 quit_event: Type[threading.Event],
                 name = 'keyboard-input-thread'):
        super().__init__(name=name, group=None)
        self.setpoint_type: SetpointType = SetpointType.NONE
        self.setpoint_val: float = None
        self.setpoint_event = new_setpoint_event
        self.quit_event = quit_event
        self.lock = lock
        self.start()

    def run(self):
        print("Input options- a: actuator_ang, v: actuator_vel, t: actuator_t, c: cam_ang, f: cable_force, h: home_pos, q: quit \n")
        while True:
            msg=input()
            f_word = msg[0]
            content = msg[1:]
            if f_word == 'a':
                self.lock.acquire()
                self.setpoint_type = SetpointType.ACTUATOR_ANGLE
                self.setpoint_val = float(content)
                self.setpoint_event.set()
                self.lock.release()
                
            elif f_word == 'v':
                self.lock.acquire()
                self.setpoint_type = SetpointType.ACTUATOR_VELOCITY
                self.setpoint_val = float(content)
                self.setpoint_event.set()
                self.lock.release()
            
            elif f_word == 't':
                self.lock.acquire()
                self.setpoint_type = SetpointType.ACTUATOR_TORQUE
                self.setpoint_val = float(content)
                self.setpoint_event.set()
                self.lock.release()

            elif f_word == 'c':
                self.lock.acquire()
                self.setpoint_type = SetpointType.CAM_ANGLE
                self.setpoint_val = float(content)
                self.setpoint_event.set()
                self.lock.release()

            elif f_word == 'f':
                self.lock.acquire()
                self.setpoint_type = SetpointType.CABLE_FORCE
                self.setpoint_val = float(content)
                self.setpoint_event.set()
                self.lock.release()
            
            elif f_word == 'h':
                self.lock.acquire()
                self.setpoint_type = SetpointType.HOME_POSITION
                self.setpoint_val = float(0)
                self.setpoint_event.set()
                self.lock.release()
            
            elif f_word == 'q':
                self.lock.acquire()
                self.quit_event.set()
                self.lock.release()
            
            else:
                print('Invalid Input, dont know yet how to process it')
