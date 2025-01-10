from ACTUATOR_CODE_ODRIVE import SpringActuator_ODrive
import filters
import enum

class SetpointType(enum):
    CABLE_FORCE = 0
    CAM_ANGLE = 1
    ACTUATOR_VELOCITY = 2
    ACTUATOR_ANGLE = 4
    ACTUATOR_TORQUE = 5
    HOME_POSITION = 6
    NONE = None

class Controller(object):
    '''Parent controller object. Child classes inherit methods.'''
    def __init__(self, actuator: SpringActuator_ODrive):
        self.actuator = actuator

    def command(self, reset):
        '''For modularity, new controllers will ideally not take any arguments with
        their command() function. The exo object stored on self will have updated
        data, which is accessible to controller objects.'''
        raise ValueError('command() not defined in child class of Controller')

    def update_controller_variables(self):
        ''' Used to update controller variables '''
        pass 

class CommandSetpoint(Controller):
    def __init__(self, actuator: SpringActuator_ODrive, setpoint_type: SetpointType = SetpointType.CAM_ANGLE, setpoint_value: float = 0):
        self.actuator = actuator
        self.butterfilter = filters.Butterworth(N=2,Wn=20,fs=200)
        self.setpoint_type = setpoint_type
        self.setpoint_value = setpoint_value
        self.tracking_status = False

    def command(self, reset=False):
        if self.setpoint_type==SetpointType.CAM_ANGLE:
            self.actuator.command_cam_angle(self.setpoint_value,error_filter=self.butterfilter)
        
        elif self.setpoint_type==SetpointType.ACTUATOR_VELOCITY:
            self.actuator.command_actuator_velocity(self.setpoint_value, mode="ramp")

        elif self.setpoint_type==SetpointType.CABLE_FORCE:
            if self.setpoint_value <= self.actuator.design_constants.CAM_DISENGAGE_FORCE_VAL:
                cam_angle = self.actuator._force_to_CAM_angle(self.setpoint_value)
                self.actuator.command_cam_angle(cam_angle)
            else:
                self.actuator.command_actuator_torque(self.setpoint_value*self.actuator.design_constants.ACTUATOR_RADIUS,mode="ramp")
        
        elif self.setpoint_type==SetpointType.HOME_POSITION:
            self.actuator.command_cam_angle(0,error_filter=self.butterfilter)
            if(self.actuator.data.cam_angle < 3): 
                self.tracking_status=True
        
        elif self.setpoint_type==SetpointType.ACTUATOR_ANGLE:
            self.actuator.command_relative_actuator_angle(self.setpoint_value, mode="step")

        elif self.setpoint_type==SetpointType.ACTUATOR_TORQUE:
            self.actuator.command_actuator_torque(self.setpoint_value, mode="step")
            
        else:
            pass

        return self.tracking_status
                    
    def update_controller_variables(self, setpoint_type, setpoint_value):
        self.butterfilter.restart()
        self.setpoint_type = setpoint_type
        self.setpoint_value = setpoint_value
        self.tracking_status = False
