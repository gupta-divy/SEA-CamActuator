from ACTUATOR_CODE_MOTEUS import SpringActuator_moteus
import filters
from enum import Enum
import numpy as np  
from scipy.interpolate import PchipInterpolator

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
        self.butterfilter = filters.Butterworth(N=2, Wn=98, fs=actuator.config.control_loop_freq)
        self.setpoint_type = SetpointType.NONE
        self.setpoint_value = 0
        self.reference_angle = 0
        self.tracking_status = False
        self.counter = 0
        self.switched = False
        self.transition_point = 0
        self.torque_filter = filters.PaddedMovingAverage(4)

        # switching_ang_pts = [69,70,71.8,72.25,73.5,74,75,75.75,76.5,78]
        # force_pts = [6,15,25,35,50,65,80,100,120,200]
        # self.pchip = PchipInterpolator(force_pts,switching_ang_pts)

    async def command(self, reset: bool = False) -> bool:
        """Issue a command to the actuator based on the current setpoint type."""
        try:
            if self.setpoint_type == SetpointType.CAM_ANGLE:
                await self.actuator.command_cam_angle(self.setpoint_value, error_filter=self.butterfilter)

            elif self.setpoint_type == SetpointType.ACTUATOR_VELOCITY:
                await self.actuator.command_actuator_velocity(self.setpoint_value)

            elif self.setpoint_type == SetpointType.CABLE_FORCE:
                
                if self.setpoint_value <= self.actuator.design_constants.CAM_DISENGAGE_FORCE_VAL:
                    self.actuator.data.commanded_cable_force = 0.3
                    cam_angle = 20
                    await self.actuator.command_cam_angle(cam_angle, error_filter=self.butterfilter)
                else:
                    # torque = self.setpoint_value * self.actuator.design_constants.ACTUATOR_RADIUS
                    # await self.actuator.command_actuator_torque(torque)
                    self.actuator.data.commanded_cable_force = self.setpoint_value
                    # switch_point = self.pchip(self.setpoint_value)
                    switch_point = 71
                    if self.actuator.data.cam_angle<switch_point and not self.switched:
                        'Transition Controller between low force regime and high force regime'
                        actuator_angle = (self.actuator.func_camAng_to_cableLen(self.transition_point)-self.actuator.func_camAng_to_cableLen(switch_point+2))/(self.actuator.design_constants.ACTUATOR_RADIUS*1000)
                        actuator_angle = actuator_angle * 180 / np.pi
                        self.torque_filter.filter(self.actuator.data.actuator_torque)
                        await self.actuator.command_relative_actuator_angle(actuator_angle, self.reference_angle)
                    elif(self.actuator.data.cam_angle>=switch_point or self.switched):
                        'When Switched to open loop torque control mode'
                        self.switched = True
                        torque = self.setpoint_value * self.actuator.design_constants.ACTUATOR_RADIUS
                        torque_smoothened = self.torque_filter.filter(torque)
                        await self.actuator.command_actuator_torque(torque_smoothened)
                    else:
                        await self.actuator.command_actuator_torque(0)

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
            if setpoint_type==SetpointType.CABLE_FORCE:
                self.switched = False
                self.transition_point = self.actuator.data.cam_angle
                self.torque_filter.restart()
            self.setpoint_type = setpoint_type
            self.setpoint_value = setpoint_value
            self.reference_angle = self.actuator.data.actuator_angle
            self.tracking_status = False
        else:
            print("Given setpoint may cause instability, so not updating")
    
    def check_input_safety(self, setpoint_type: SetpointType, setpoint_val: float) -> bool:
        """Check if the given setpoint is safe to use."""
        return True