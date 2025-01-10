import odrive
from odrive.enums import *
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
    MAX_DC_OVERVOLTAGE_TRIP_LEVEL = 26
    MIN_DC_OVERVOLTAGE_TRIP_LEVEL = 12

def setControllerConfiguration(odrv):
    '''Set controller values on ODrive reading from controller class'''
    control = ControllerConfigurationValues()
    odrv.axis0.controller.config.vel_ramp_rate = control.VEL_RAMP_RATE
    odrv.axis0.controller.config.torque_ramp_rate = control.TORQUE_RAMP_RATE
    odrv.axis0.trap_traj.config.accel_limit = control.TRAP_TRAJ_ACCEL_LIM
    odrv.axis0.trap_traj.config.decel_limit = control.TRAP_TRAJ_DECCEL_LIM
    odrv.axis0.trap_traj.config.vel_limit = control.TRAP_TRAJ_VEL_LIM
    odrv.axis0.controller.config.absolute_setpoints = control.USE_ABSOLUTE_MOTOR_ANGLE_FOR_SETPOINT    
    pass

def setCoreConfiguration(odrv):
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
    
def fullCalibration(odrv):
    odrv.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    time.sleep(15)

def isOdriveConfigurationValid(odrv):
    '''Validate ODrive configurable variable match with tested values in config file'''
    config = CoreConfigurationValues()
    if (odrv.axis0.config.motor.motor_type == config.MOTOR_TYPE and
        odrv.axis0.config.motor.pole_pairs == config.MOTOR_POLE_PAIRS and
        odrv.axis0.config.motor.current_soft_max == config.MOTOR_CURRENT_SOFT_MAX and
        odrv.config.dc_bus_overvoltage_trip_level == config.MAX_DC_OVERVOLTAGE_TRIP_LEVEL and
        odrv.config.dc_bus_undervoltage_trip_level == config.MIN_DC_OVERVOLTAGE_TRIP_LEVEL and
        odrv.config.dc_max_positive_current == config.MAX_POSITIVE_DC_CURRENT and
        odrv.config.dc_max_negative_current == config.MAX_NEGATIVE_DC_CURRENT and
        odrv.axis0.config.load_encoder == config.MOTOR_ENCODER and
        odrv.axis0.config.commutation_encoder == config.MOTOR_ENCODER and
        odrv.rs485_encoder_group0.config.mode == config.MOTOR_ENCODER_MODEL):
        print("Core Config are correct!")   
    else:
        print("Core Config are not correct!")
    
    check_torque_constent = odrv.axis0.config.motor.torque_constant
    check_calibration_current = odrv.axis0.config.motor.calibration_current
    print(check_torque_constent) 
    print(check_calibration_current)
    False

if __name__ == "__main__":
    odrv = odrive.find_any()
    setCoreConfiguration(odrv)
    setControllerConfiguration(odrv)  
    fullCalibration(odrv)
    isOdriveConfigurationValid(odrv)