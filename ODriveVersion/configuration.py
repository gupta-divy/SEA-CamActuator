import odrive
from odrive.enums import *

class ControllerConfigurationValues():
    #VELOCITY_CONTROL
    VEL_RAMP_RATE = 100
    
    #TORQUE_CONTROL
    TORQUE_RAMP_RATE = 2
    
    #POSITION_CONTROL
    TRAP_TRAJ_ACCEL_LIM = 2
    TRAP_TRAJ_DECCEL_LIM = 2
    TRAP_TRAJ_VEL_LIM = 100

    # Reference Frame
    USE_ABSOLUTE_MOTOR_ANGLE_FOR_SETPOINT = False
    

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

def setControllerConfiguration(odrv: odrive.Odrive):
    control = ControllerConfigurationValues()
    odrv.axis0.controller.config.absolute_setpoints = control.USE_ABSOLUTE_MOTOR_ANGLE_FOR_SETPOINT

def setCoreConfiguration(odrv: odrive.Odrive):
    '''Set configurable variables on ODrive reading from config class'''
    config = CoreConfigurationValues() 
    odrv.axis0.config.motor.motor_type = config.MOTOR_TYPE
    odrv.axis0.config.motor.pole_pairs = config.MOTOR_POLE_PAIRS
    odrv.axis0.config.motor.torque_constant = config.MOTOR_TORQUE_CONSTANT
    odrv.axis0.config.motor.current_soft_max = config.MOTOR_CURRENT_SOFT_MAX
    odrv.axis0.config.motor.calibration_current = config.MOTOR_CALIBRATION_CURRENT
    odrv.config.dc_bus_overvoltage_trip_level = config.MAX_DC_OVERVOLTAGE_TRIP_LEVEL
    odrv.config.dc_bus_undervoltage_trip_level = config.MIN_DC_OVERVOLTAGE_TRIP_LEVEL
    odrv.config.dc_max_positive_current = config.MAX_POSITIVE_DC_CURRENT
    odrv.config.dc_max_negative_current = config.MAX_NEGATIVE_DC_CURRENT
    odrv.axis0.config.load_encoder = config.MOTOR_ENCODER
    odrv.axis0.config.commutation_encoder = config.MOTOR_ENCODER
    odrv.rs485_encoder_group0.config.mode = config.MOTOR_ENCODER_MODEL
    odrv.save_configuration()
    odrv.reboot()

def isOdriveConfigurationValid():
    '''Validate ODrive configurable variable match with tested values in config file'''
    False

if __name__ == "__main__":
    odrv = odrive.find_any()
    setCoreConfiguration(odrv)