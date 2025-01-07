import odrive
from odrive.enums import *
import math
import time

class ControllerConfigurationValues():
    #VELOCITY_CONTROL
    VEL_RAMP_RATE = 100
    
    #TORQUE_CONTROL
    TORQUE_RAMP_RATE = 2
    
    #POSITION_CONTROL
    TRAP_TRAJ_ACCEL_LIM = 2
    TRAP_TRAJ_DECCEL_LIM = 2
    TRAP_TRAJ_VEL_LIM = 100
    

class CoreConfigurationValues():
    # Motor Configuration
    MOTOR_TYPE = MotorType.HIGH_CURRENT
    MOTOR_POLE_PAIRS = 14
    MOTOR_KV = 80
    MOTOR_TORQUE_CONSTANT = 8.27 / MOTOR_KV
    MOTOR_CALIBRATION_CURRENT = 4.8             # rated current
    MOTOR_CURRENT_SOFT_MAX = 12.5               # motor peak current
    
    # Encoder Configuration
    MOTOR_ENCODER = EncoderId.RS485_ENCODER0
    MOTOR_ENCODER_MODEL = Rs485EncoderMode.ODRIVE_OA1

    # Power Configuration
    MAX_POSITIVE_DC_CURRENT = 6
    MAX_NEGATIVE_DC_CURRENT = -6
    MAX_DC_OVERVOLTAGE_TRIP_LEVEL = 24
    MIN_DC_OVERVOLTAGE_TRIP_LEVEL = 12

    # Reference Frame
    USE_ABSOLUTE_MOTOR_ANGLE_FOR_SETPOINT = False

def setControllerConfiguration(odrv: odrive.Odrive):
    pass

def setCoreConfiguration(odrv: odrive.Odrive):
    '''Set configurable variables on ODrive reading from config class'''
    config = CoreConfigurationValues()
    odrv.axis0.controller.config.absolute_setpoints = config.USE_ABSOLUTE_MOTOR_ANGLE_FOR_SETPOINT
    odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
    odrv.axis0.config.motor.pole_pairs = 14
    odrv.axis0.config.motor.torque_constant = 0.103375
    odrv.axis0.config.motor.current_soft_max = 12
    odrv.axis0.config.motor.current_hard_max = 25.6
    odrv.axis0.config.motor.calibration_current = 5
    odrv.axis0.config.motor.resistance_calib_max_voltage = 2
    odrv.axis0.config.calibration_lockin.current = 10
    odrv.axis0.motor.motor_thermistor.config.enabled = False
    odrv.axis0.config.torque_soft_min = -math.inf
    odrv.axis0.config.torque_soft_max = math.inf
    odrv.config.dc_bus_overvoltage_trip_level = 24
    odrv.config.dc_bus_undervoltage_trip_level = 12
    odrv.config.dc_max_positive_current = 6
    odrv.config.dc_max_negative_current = -6

    odrv.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
    odrv.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0
    odrv.rs485_encoder_group0.config.mode = Rs485EncoderMode.ODRIVE_OA1
    
    odrv.save_configuration()
    odrv.reboot()

def isOdriveConfigurationValid():
    '''Validate ODrive configurable variable match with tested values in config file'''
    False

if __name__ == "__main__":
    odrv = odrive.find_any()
    setCoreConfiguration(odrv)
    
    
    



