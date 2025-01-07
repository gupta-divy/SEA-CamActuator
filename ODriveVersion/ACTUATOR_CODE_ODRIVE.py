import filters
from scipy.interpolate import pchip_interpolate
import numpy as np
from math import sin, cos, radians, sqrt
import traceback
from dataclasses import dataclass
import time
import csv
import odrive
from odrive.enums import ControlMode, InputMode, AxisState

def setMotorConfiguration(odrv: odrive.Odrive):
    '''Set configurable variables on ODrive reading from config file'''
    # No config file yet so writing important ones for now - 05/01/2025
    odrv.axis0.motor.config.torque_constant = 0.13
    odrv.axis0.controller.config.absolute_setpoints = False
    # set external encoder to be used for motor and inbuilt encoder for CAM angle
    # set controller configuration for different methods
    # see, if we wanna use gain scheduling

def isMotorConfigurationValid():
    '''Validate ODrive configurable variable match with tested values in config file'''
    # Variables do not get overwritten on Odrive so that's why no need to set configuration everytime
    # Need to test and write a config file
    False

class Config:
    control_loop_freq = 200
    camControllerGainKp = 4
    camControllerGainKd = 0.2
    calibrationVelocity = 5     # deg / sec
    calibrationTime = 5         # sec

class Constants:
    MAX_ALLOWABLE_VOLTAGE_COMMAND = 3000                        # mV
    MAX_ALLOWABLE_CURRENT = 5000                                # mA
    MOTOR_TO_ACTUATOR_TR = 8
    MOTOR_REV_TO_ACTUATOR_DEG = 360 / MOTOR_TO_ACTUATOR_TR      # Deg / Revolution
    CAM_REV_TO_DEG = 360                                        # Deg / Revolution 
    MOTOR_KV = 80                                               # RPM/V
    MOTOR_CURRENT_TO_MOTOR_TORQUE = 8.27 / MOTOR_KV             # From ODrive Manual, use with MotorCurrent in Amps
    MOTOR_TORQUE_CONSTANT = 0.13                                # Nm/A
    MS_TO_SECONDS = 0.001
    default_KP = 300                                            # Motor Gains
    default_KI = 350                                            # Motor Gains
    default_KD = 0                                              # Motor Gains
    default_KVAL = 0                                            # Not changing from default
    default_BVAL = 0                                            # Not changing from default
    default_FF = 0

class DesignConstants:
    'Actuator Design Constants, with measurements in mm and degrees with reference at CAM center'
    ACTUATOR_RADIUS = 37.5
    ROLLER_RADIUS = 4.5
    CAM_LEVER_ARM = 55
    ROLLER_A_CORD = (-36.14,61.24)
    ROLLER_B_CORD = (-23.66,18)
    ROLLER_C_INIT_ANG = 59
    CAM_RANGE = 75  
    INIT_CABLE_LEN_BW_ANKLE_ACT = 94.5
    CAM_SIGN = 1

class SpringActuator_ODrive:
# ACTUATOR INITIALIZATION AND DATA FUNCTIONS
    def __init__(self, odrv: odrive.Odrive, dataFile_name: str, motor_serial: str):
        self.odrv = odrv
        self.data = self.DataContainer()
        self.constants = Constants()
        self.design_constants = DesignConstants()
        self.config = Config()
        self.actuator_offset = None                    # post-calibration zero reference actuator angle
        self.cam_offset = None                      # post-calibration zero reference cam angle
        self.has_calibrated = False
        self.dataFile_name = dataFile_name
        self.setup_data_writer(dataFile_name)
        self.motor_sign = 1                         # motor_serial to be used to update motor_sign if required

    #DATA LOGGING AND STORING METHODS
    @dataclass
    class DataContainer:
        '''A nested dataclass within Actuator, reserving space for instantaneous data.'''
        loop_time: float = None
        motor_current: float = 0
        motor_voltage: float = 0
        motor_mechanical_power: float = 0
        motor_electrical_power: float = 0
        actuator_angle: float = 0
        actuator_velocity: float = 0
        actuator_torque: float = 0
        cam_angle: float = 0
        commanded_actuator_angle: float = None
        commanded_actuator_velocity: float = None
        commanded_actuator_torque: float = None
        commanded_cam_angle: float = None

        # Extra logged values for use with Exoskeleton
        exo_angle_estimate: float = None
        exo_velocity_estimate: float = None
        exo_torque_estimate: float = None
        commanded_exo_torque: float = None

        # Extra logged values for Debug
        cam_angle_error: float = None
        cam_angle_error_integral: float = None

    def read_data(self, loop_time=None):
        '''Read data from ODrive, store in ActpackMech.data : Data Container.'''
        self.data.loop_time = loop_time
        self.data.motor_current = self.odrv.axis0.motor.foc.Iq_measured
        self.data.motor_voltage = 0 # Not sure how to read the motor voltage, vbus has dc voltage + brake resistor voltage
        self.data.motor_mechanical_power = self.odrv.axis0.motor.mechanical_power
        self.data.motor_electrical_power = self.odrv.axis0.motor.electrical_power
        self.data.actuator_angle = self.odrv.axis0.pos_estimate * self.constants.MOTOR_REV_TO_ACTUATOR_DEG
        self.data.actuator_velocity = self.odrv.axis0.vel_estimate * self.constants.MOTOR_REV_TO_ACTUATOR_DEG
        self.data.actuator_torque = self.axis0.motor.torque_estimate * self.constants.MOTOR_TO_ACTUATOR_TR
        self.data.cam_angle = 0 # Not sure how to read or select second encoder for this
        if(self.cam_offset!=None): self.data.cam_angle = self.data.cam_angle - self.cam_offset


        self.data.exo_angle_estimate = None             # Need to define helper function for this
        self.data.exo_velocity_estimate = None          # Need to define helper function for this
        self.data.exo_torque_estimate = None            # Need to define helper function for this
        self.data.commanded_exo_torque = None
        
    def write_data(self):
        '''Writes data file, only if new data packet is available'''
        if self.dataFile_name is not None:
            self.writer.writerow(self.data.__dict__)
    
    def close_file(self):
        if self.dataFile_name is not None:
            self.my_file.close()

    def setup_data_writer(self, file_ID: str):
        if file_ID is not None:
            '''file_ID is used as a custom file identifier after date.'''
            # subfolder_name = 'exo_data/'
            self.filename = time.strftime("%Y%m%d_%H%M_") + file_ID + '.csv'
            self.my_file = open(self.filename, 'w', newline='')
            self.writer = csv.DictWriter(
                self.my_file, fieldnames=self.data.__dict__.keys())
            self.writer.writeheader()

# COMMANDING CONTROLLER FUNCTIONS
    def command_relative_actuator_angle(self, des_rel_angle: float, mode: str="step"):
        '''
        des_rel_angle: Takes in angle (degrees) to rotate actuator from current position
        mode: setpoint method for position controller,  DEFAULT = step_input
        mode options: Step Input = step, 2nd Order Filtered = filt, Trapezoidal Velocity Profile = trap 
        '''
        # filt and trap can be configured, check Odrive InputMode API reference
        des_input_mode = [InputMode.PASSTHROUGH if mode=="step" else 
                          InputMode.POS_FILTER if mode=="filt" else
                          InputMode.TRAP_TRAJ if mode=="trap" else
                          InputMode.PASSTHROUGH]
        
        if(self.odrv.axis0.controller.control_mode != ControlMode.POSITION_CONTROL):
            self.odrv.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
        
        if(self.odrv.axis0.controller.input_mode != des_input_mode):
            self.odrv.axis0.controller.config.input_mode = des_input_mode
        
        desired_motor_pos = (des_rel_angle + self.data.actuator_angle) / self.constants.MOTOR_REV_TO_ACTUATOR_DEG
        self.odrv.axis0.controller.input_pos = desired_motor_pos
    
    def command_actuator_velocity(self, des_velocity: float, mode: str="step"):
        '''
        des_velocity: Takes in velocity command in degrees/seconds
        mode: setpoint method for velocity controller,  DEFAULT = step_input
        mode options: Step Input = step, ramped Velocity to desired vel = ramp
        '''
        des_input_mode = [InputMode.PASSTHROUGH if mode=="step" else 
                          InputMode.VEL_RAMP if mode=="ramp" else
                          InputMode.PASSTHROUGH]
        
        if(self.odrv.axis0.controller.control_mode != ControlMode.VELOCITY_CONTROL):
            self.odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        
        if(self.odrv.axis0.controller.input_mode != des_input_mode):
            self.odrv.axis0.controller.config.input_mode = des_input_mode
        
        desired_motor_vel = des_velocity / self.constants.MOTOR_REV_TO_ACTUATOR_DEG
        self.odrv.axis0.controller.input_vel = desired_motor_vel
    
    def command_actuator_torque(self, des_torque: float, mode: str = "step"):
        '''
        des_torque: Takes in torque command in Nm for actuator
        mode: setpoint method for torque controller,  DEFAULT = step_input
        mode options: Step Input = step, ramped torque to desired = ramp
        '''
        des_input_mode = [InputMode.PASSTHROUGH if mode=="step" else 
                          InputMode.TORQUE_RAMP if mode=="ramp" else
                          InputMode.PASSTHROUGH]
        
        if(self.odrv.axis0.controller.control_mode != ControlMode.TORQUE_CONTROL):
            self.odrv.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        
        if(self.odrv.axis0.controller.input_mode != des_input_mode):
            self.odrv.axis0.controller.config.input_mode = des_input_mode
        
        desired_motor_torque = des_torque / self.constants.MOTOR_TO_ACTUATOR_TR
        self.odrv.axis0.controller.input_torque = desired_motor_torque

    def command_cam_angle(self, des_angle: float, filter: filters.Filter):
        prev_cam_ang_err = self.data.cam_angle_error
        curr_cam_ang_err = des_angle - self.data.cam_angle
        curr_cam_ang_err_diff = (curr_cam_ang_err - prev_cam_ang_err) * self.config.control_loop_freq
        curr_cam_ang_err_diff = filter.filter(curr_cam_ang_err_diff)
        des_act_vel = self.config.camControllerGainKp * curr_cam_ang_err + self.config.camControllerGainKd * curr_cam_ang_err_diff
        self.command_actuator_velocity(des_velocity=des_act_vel, mode="step")
    
    def command_controller_off(self):
        # Not sure how to implement this
        pass
    
# MECHANISM INITIAL CALIBRATION
    def intial_calibration(self):
        try:
            '''Brings up slack, calibrates ankle and motor offset angles.'''
            input('Press Enter to calibrate exo on')
            print('Calibrating...')
            cam_ang_filter = filters.MovingAverage(window_size=10)
            t0 = time.time()
            
            self.command_actuator_velocity(-1* self.motor_sign * self.config.calibrationVelocity)
            while time.time()-t0 < self.config.calibrationTime:
                last_read_CAM_val = self.data.cam_angle
                time.sleep(1/self.config.control_loop_freq)
                self.read_data()
                if(abs(cam_ang_filter(self.data.cam_angle)-last_read_CAM_val) < 1):
                    self.cam_offset = self.data.cam_angle
                    break
                
            self.command_actuator_velocity(0)
            time.sleep(0.1)

            self.command_actuator_velocity(self.motor_sign * self.config.calibrationVelocity)
            while time.time()-t0 < self.config.calibrationTime:
                time.sleep(1/self.config.control_loop_freq)
                self.read_data()
                if(cam_ang_filter(self.data.cam_angle) > 5):
                    self.actuator_offset = self.data.actuator_angle
                    self.has_calibrated = True
                    break
            
            self.command_actuator_velocity(0)

            if not(self.has_calibrated):
                raise RuntimeError('Calibration Timed Out!')
            
            self.read_data()
            print("CAM Angle: ", self.data.cam_angle)
            print('CAM Offset: ', self.cam_offset)
            print('Motor offset: ', self.actuator_offset)        
            print('Finished Calibrating')

        except Exception as err:
            print(traceback.print_exc())
            self.close()
            raise RuntimeError('Calibration Failed') 

    def close(self):
        self.odrv.axis0.AxisState = AxisState.IDLE
        self.close_file()

def connect_to_actuator(dataFile_name: str):
    '''Connect to Actuator, instantiate Actuator object'''
    # Connection settings
    try:
        odrv: odrive.Odrive = odrive.find_any()
        device_serial: str = hex(odrv.serial_number).upper()
        odrv.axis0.AxisState = AxisState.CLOSED_LOOP_CONTROL
        print("Connected to Odrive: ", device_serial)
    except Exception as err:
        traceback.print_exc()
        raise RuntimeError('Unable to connect to Odrive, Check Connection')

    if not isMotorConfigurationValid():
        setMotorConfiguration()

    actuator = SpringActuator_ODrive(odrv, dataFile_name, device_serial)
    return actuator
