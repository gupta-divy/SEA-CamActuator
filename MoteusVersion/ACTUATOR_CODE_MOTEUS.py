import filters
import asyncio
from scipy.interpolate import pchip_interpolate
import math
import traceback
from dataclasses import dataclass
import time
import csv
import moteus
from moteus import multiplex as moteusMp
from enum import Enum

class moteusDataMap(Enum):
    MODE = 0
    POSITION = 1
    VELOCITY = 2
    TORQUE = 3
    POWER = 7
    VOLTAGE = 13
    TEMPERATURE = 14
    FAULT = 15
    ENC2_POSITION = 84
    ENC2_VELOCITY = 85
    CONTROLLER_CLOCK = 112

class ControllerConfig:
    control_loop_freq = 200             # hertz
    camControllerGainKp = 4.0           
    camControllerGainKd = 0.0           
    calibrationVelocity = 60            # deg / sec
    calibrationTime = 5                 # sec
    calibrationCamThreshold = 5         # deg
    actuatorVelocitySaturation = 2000    # deg / sec
    actuatorTorqueSaturation = 3        # Nm 
    homeAngleThreshold = 5

class Constants:
    MAX_ALLOWABLE_VOLTAGE_COMMAND = 3000                        # mV
    MAX_ALLOWABLE_CURRENT = 5000                                # mA
    MOTOR_TO_ACTUATOR_TR = 8
    MOTOR_POS_EST_TO_ACTUATOR_DEG = 360 / MOTOR_TO_ACTUATOR_TR  # Deg / Revolution / Transmission ratio
    CAM_ENC_TO_DEG = 360                                        # Deg / Revolution
    MS_TO_SECONDS = 0.001
    SPLINE_A_PTS_FORCE_ANGLE_CONVERSION = [0,10,20,40,70,80,85]
    SPLINE_F_PTS_FORCE_ANGLE_CONVERSION = [2.15,2.2,2.275,2.4,6.5,9.5,11]

class DesignConstants:
    'Actuator Design Constants, with measurements in mm and degrees with reference at CAM center'
    ACTUATOR_RADIUS = 0.0375 
    ROLLER_RADIUS = 0.0045
    CAM_LEVER_ARM = 0.055
    ROLLER_A_CORD = (-36.14,61.24)
    ROLLER_B_CORD = (-23.66,18)
    ROLLER_C_INIT_ANG = 59
    CAM_RANGE = 75  
    INIT_CABLE_LEN_BW_ANKLE_ACT = 0.0945
    CAM_DISENGAGE_FORCE_VAL = 8

class SpringActuator_moteus:
# ACTUATOR INITIALIZATION AND DATA FUNCTIONS
    def __init__(self, moteus_motor_ctrl, dataFile_name: str):
        self.motor_ctrl: moteus.Controller = moteus_motor_ctrl
        self.data = self.DataContainer()
        self.constants = Constants()
        self.design_constants = DesignConstants()
        self.config = ControllerConfig()
        self.actuator_offset = None                 # post-calibration zero reference actuator angle
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
        mc_input_voltage: float = 0
        mc_power: float = 0
        mc_clock: int = 0
        mc_temperature: int = 0
        mc_fault: int = 0
        mc_mode: int = 0 
        actuator_angle: float = 0
        actuator_velocity: float = 0
        actuator_torque: float = 0
        cam_encoder_raw: int = 0
        cam_angle: float = 0
        cam_velocity: float = 0
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

    async def read_data(self, loop_time=None):
        '''Read data from Moteus, store in Data Container.'''
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
                moteus.Register.MILLISECOND_COUNTER: moteusMp.INT16
            }
        data_feed = await self.motor_ctrl.custom_query(to_query)
        mc_data = data_feed.values
        self.data.loop_time = loop_time
        self.data.mc_clock = mc_data[moteusDataMap.CONTROLLER_CLOCK.value]
        self.data.mc_input_voltage = mc_data[moteusDataMap.VOLTAGE.value]
        self.data.mc_power = mc_data[moteusDataMap.POWER.value]
        self.data.mc_fault = mc_data[moteusDataMap.FAULT.value]
        self.data.mc_temperature = mc_data[moteusDataMap.TEMPERATURE.value]
        self.data.mc_mode = mc_data[moteusDataMap.MODE.value]
        self.data.actuator_angle = mc_data[moteusDataMap.POSITION.value] * self.constants.MOTOR_POS_EST_TO_ACTUATOR_DEG
        self.data.actuator_velocity = mc_data[moteusDataMap.VELOCITY.value] * self.constants.MOTOR_POS_EST_TO_ACTUATOR_DEG
        self.data.actuator_torque = mc_data[moteusDataMap.TORQUE.value] * self.constants.MOTOR_TO_ACTUATOR_TR
        
        # CAM Angle
        self.data.cam_velocity = mc_data[moteusDataMap.ENC2_VELOCITY.value] * self.constants.CAM_ENC_TO_DEG
        last_cam_encoder_raw = self.data.cam_encoder_raw if self.data.cam_encoder_raw is not None else None
        self.data.cam_encoder_raw = mc_data[moteusDataMap.ENC2_POSITION.value] * self.constants.CAM_ENC_TO_DEG
        cam_angle_jump = (self.data.cam_encoder_raw - last_cam_encoder_raw) if last_cam_encoder_raw is not None else 0
        if cam_angle_jump > 180:  # jump of pi in degrees (from 0 to 2pi as angle increases), though shouldn't be an issue as per current configuration 
            cam_angle_wrapped = self.data.cam_encoder_raw - 360.0
        else:
            cam_angle_wrapped = self.data.cam_encoder_raw
        self.data.cam_angle = -1*(cam_angle_wrapped - self.cam_offset) if self.cam_offset != None else None
        
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
    # TODO: Read usage modes on moteus to see Kp_scale and k_scale usecase for improved controller performance
    async def command_relative_actuator_angle(self, des_rel_angle: float, reference_angle: float):
        '''
        des_rel_angle: Takes in angle (degrees) to rotate actuator from current position
        '''        
        desired_motor_pos = (des_rel_angle + reference_angle) / self.constants.MOTOR_POS_EST_TO_ACTUATOR_DEG
        await self.motor_ctrl.set_position(position=desired_motor_pos, query = False)

    
    async def command_actuator_velocity(self, des_velocity: float):
        '''
        des_velocity: Takes in velocity command in degrees/seconds
        '''
        # Velocity Saturation
        des_velocity = min(des_velocity, self.config.actuatorVelocitySaturation) if des_velocity>0 else max(des_velocity, -self.config.actuatorVelocitySaturation)
        desired_motor_vel = des_velocity / self.constants.MOTOR_POS_EST_TO_ACTUATOR_DEG
        await self.motor_ctrl.set_position(position=math.nan, velocity=desired_motor_vel)
    
    async def command_actuator_torque(self, des_torque: float):
        '''
        des_torque: Takes in torque command in Nm for actuator
        '''
        # Torque Saturation
        des_torque = min(des_torque, self.config.actuatorTorqueSaturation)
        desired_motor_torque = des_torque / self.constants.MOTOR_TO_ACTUATOR_TR
        await self.motor_ctrl.set_position(torque = desired_motor_torque)

    async def command_cam_angle(self, des_angle: float, error_filter: filters.Filter):  
        prev_cam_ang_err = self.data.cam_angle_error
        if prev_cam_ang_err is None:
            prev_cam_ang_err = 0.0
        curr_cam_ang_err = des_angle - self.data.cam_angle
        curr_cam_ang_err_diff = (curr_cam_ang_err - prev_cam_ang_err) * self.config.control_loop_freq
        curr_cam_ang_err_diff = error_filter.filter(curr_cam_ang_err_diff)
        des_act_vel = self.config.camControllerGainKp * curr_cam_ang_err + self.config.camControllerGainKd * curr_cam_ang_err_diff
        
        await self.command_actuator_velocity(des_velocity=des_act_vel)
    
    async def command_controller_off(self):
        await self.motor_ctrl.set_stop()

# MECHANISM INITIAL CALIBRATION
    async def initial_calibration(self):
        try:
            print('Calibrating...')
            stability_window_size=20
            stability_window = []
            for i in range(int(self.config.calibrationTime * self.config.control_loop_freq)):
                await self.command_actuator_velocity(-1 * self.motor_sign * self.config.calibrationVelocity)
                await self.read_data()
                stability_window.append(self.data.cam_encoder_raw)
                if len(stability_window) > stability_window_size:
                    stability_window.pop(0)
                if i>stability_window_size and abs((max(stability_window) - min(stability_window))) < 0.5:
                    self.cam_offset = self.data.cam_encoder_raw
                    break
                time.sleep(1/self.config.control_loop_freq)
            await self.command_actuator_velocity(0)
            for _ in range(int(self.config.calibrationTime * self.config.control_loop_freq)):
                await self.read_data()
                await self.command_actuator_velocity(self.motor_sign * self.config.calibrationVelocity)
                if self.data.cam_angle > self.config.calibrationCamThreshold:
                    self.actuator_offset = self.data.actuator_angle
                    self.has_calibrated = True
                    break
                time.sleep(1/self.config.control_loop_freq)
            
            await self.command_actuator_velocity(0)

            if not self.has_calibrated:
                raise RuntimeError('Calibration Timed Out!')
            
            print(f"CAM Angle: {self.data.cam_angle}, CAM Offset: {self.cam_offset}, Actuator Offset: {self.actuator_offset}")
            print('Finished Calibrating')

        finally:
            await self.command_controller_off()

    async def close(self):
        await self.command_controller_off()
        self.close_file()

# Helper functions for conversion across different measurements
    def _force_to_CAM_angle(self,cable_force):
        return pchip_interpolate(self.constants.SPLINE_F_PTS_FORCE_ANGLE_CONVERSION, self.constants.SPLINE_A_PTS_FORCE_ANGLE_CONVERSION, cable_force)

async def connect_to_actuator(dataFile_name: str):
    '''Connect to Actuator, instantiate Actuator object'''
    # Connection settings
    try:
        moteus_controller = moteus.Controller()
        await moteus_controller.set_stop()
    except Exception as err:
        traceback.print_exc()
        raise RuntimeError('Unable to connect to motor controller, Check Connection')

    print("Connected to Moteus")
    
    actuator = SpringActuator_moteus(moteus_controller, dataFile_name)
    return actuator
