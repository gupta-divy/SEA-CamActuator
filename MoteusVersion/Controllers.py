from ACTUATOR_CODE_MOTEUS import SpringActuator_moteus
import filters
from enum import Enum

class SetpointType(Enum):
    NONE = 0
    CAM_ANGLE = 1
    ACTUATOR_VELOCITY = 2
    ACTUATOR_ANGLE = 4
    ACTUATOR_TORQUE = 5
    HOME_POSITION = 6
    CABLE_FORCE = 7

class Controller(object):
    """Parent controller object. Child classes inherit methods."""
    def __init__(self, actuator: SpringActuator_moteus):
        self.actuator = actuator

    def command(self, reset: bool):
        """Override this method in child classes to define control logic."""
        raise ValueError("command() not defined in child class of Controller")

    def update_controller_variables(self):
        """Override this method in child classes to update controller variables."""
        pass 

class CommandSetpoint(Controller):
    def __init__(self, actuator: SpringActuator_moteus):
        super().__init__(actuator)
        self.butterfilter = filters.Butterworth(N=2, Wn=20, fs=200)
        self.setpoint_type = SetpointType.NONE
        self.setpoint_value = 0
        self.reference_angle = 0
        self.tracking_status = False

    async def command(self, reset: bool = False) -> bool:
        """Issue a command to the actuator based on the current setpoint type."""
        try:
            if self.setpoint_type == SetpointType.CAM_ANGLE:
                await self.actuator.command_cam_angle(self.setpoint_value, error_filter=self.butterfilter)

            elif self.setpoint_type == SetpointType.ACTUATOR_VELOCITY:
                await self.actuator.command_actuator_velocity(self.setpoint_value)

            elif self.setpoint_type == SetpointType.CABLE_FORCE:
                if self.setpoint_value <= self.actuator.design_constants.CAM_DISENGAGE_FORCE_VAL:
                    cam_angle = self.actuator._force_to_CAM_angle(self.setpoint_value)
                    await self.actuator.command_cam_angle(cam_angle)
                else:
                    torque = self.setpoint_value * self.actuator.design_constants.ACTUATOR_RADIUS
                    await self.actuator.command_actuator_torque(torque)

            elif self.setpoint_type == SetpointType.HOME_POSITION:
                await self.actuator.command_cam_angle(4.8, error_filter=self.butterfilter)
                if self.actuator.data.cam_angle < self.actuator.config.homeAngleThreshold:
                    self.tracking_status = True

            elif self.setpoint_type == SetpointType.ACTUATOR_ANGLE:
                await self.actuator.command_relative_actuator_angle(self.setpoint_value, self.reference_angle)

            elif self.setpoint_type == SetpointType.ACTUATOR_TORQUE:
                await self.actuator.command_actuator_torque(self.setpoint_value)

            elif self.setpoint_type == SetpointType.NONE:
                await self.actuator.command_relative_actuator_angle(0, self.actuator.data.actuator_angle)

            else:
                raise ValueError(f"Unsupported setpoint type: {self.setpoint_type}")

        except Exception as e:
            print(f"Error during command execution: {e}")
        
        return self.tracking_status
                    
    def update_controller_variables(self, setpoint_type: SetpointType, setpoint_value: float):
        """Update the controller variables with safety checks."""
        if self.check_input_safety(setpoint_type=setpoint_type, setpoint_val=setpoint_value):
            if setpoint_type != self.setpoint_type:
                self.butterfilter.restart()
            print(f"Updating Controller Variables: {setpoint_type} : {setpoint_value}")
            self.setpoint_type = setpoint_type
            self.setpoint_value = setpoint_value
            self.reference_angle = self.actuator.data.actuator_angle
            self.tracking_status = False
        else:
            print("Given setpoint may cause instability, so not updating")
    
    def check_input_safety(self, setpoint_type: SetpointType, setpoint_val: float) -> bool:
        """Check if the given setpoint is safe to use."""
        # if setpoint_type == SetpointType.ACTUATOR_TORQUE and abs(setpoint_val) > self.actuator.design_constants.MAX_TORQUE:
        #     print("Warning: Torque exceeds safe limit!")
        #     return False
        # if setpoint_type == SetpointType.ACTUATOR_VELOCITY and abs(setpoint_val) > self.actuator.design_constants.MAX_VELOCITY:
        #     print("Warning: Velocity exceeds safe limit!")
        #     return False
        # Add more checks as needed
        return True
