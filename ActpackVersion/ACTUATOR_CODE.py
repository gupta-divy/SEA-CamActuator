from flexsea import fxEnums as fxe
from flexsea import flexsea as flex
import filters
from scipy.interpolate import pchip_interpolate
import numpy as np
from math import sin, cos, radians, sqrt
import traceback
from dataclasses import dataclass
import time
import csv

fxs = flex.FlexSEA()
class Constants:
    DEFAULT_BAUD_RATE = 230400
    ACTPACK_FREQ = 200 #Hz
    MAX_ALLOWABLE_VOLTAGE_COMMAND = 3000 #mV
    MAX_ALLOWABLE_CURRENT = 5000 #mA
    MOT_ENC_CLICKS_TO_DEG = 1/(2**14/360)
    CAM_ENC_CLICKS_TO_DEG = 1/(2**14/360)
    CAM_ENC_PEAK = 16384
    MOTOR_CURRENT_TO_MOTOR_TORQUE = 0.000146
    MOTOR_TO_ACTUATOR_TR = 9
    MS_TO_SECONDS = 0.001
    default_KP = 300
    default_KI = 350
    default_KD = 0
    default_KVAL = 0 #Not changing from default
    default_BVAL = 0 #Not changing from default
    default_FF = 0
    ## Have to experimentally test
    CAM_KP = 10
    CAM_KI = 10
    CAM_KD = 1

class act_design_constants:
    'Actuator Design Constants, with measurements in mm and degrees with reference at CAM center'
    ACTUATOR_RADIUS = 37.5
    ROLLER_RADIUS = 4.5
    CAM_LEVER_ARM = 55
    ROLLER_A_CORD = (-36.14,61.24)
    ROLLER_B_CORD = (-23.66,18)
    ROLLER_C_INIT_ANG = 59
    CAM_RANGE = 75  
    INIT_CABLE_LEN_BW_ANKLE_ACT = 94.5
    CAM_ENC_INIT = 14000

class actpack_mech:
    def __init__(self, dev_id: int, dataFile_name):
        self.dev_id = dev_id
        self.data = self.DataContainer()
        self.constants = Constants()
        self.design_const = act_design_constants()
        self.motor_offset = None
        self.cam_offset = None
        self.last_state_time = None
        self.prev_motor_angle = 0
        self.has_calibrated = False
        self.unwrapped_cable = 0
        self.dataFile_name = dataFile_name
        self.setup_data_writer(dataFile_name)
        self.Kp = self.constants.default_KP
        self.Ki = self.constants.default_KI
        self.Kd = self.constants.default_KD
        self.k_val = self.constants.default_KVAL
        self.b_val = self.constants.default_BVAL
        self.ff = self.constants.default_FF
        self.motor_sign = 1

    def close(self):
        self.update_gains()
        self.command_current(desired_mA=0)
        time.sleep(0.1)
        self.command_controller_off()
        time.sleep(0.05)
        fxs.stop_streaming(self.dev_id)
        time.sleep(0.2)
        fxs.close(self.dev_id)
        self.close_file()

    #DATA LOGGING AND STORING METHODS
    @dataclass
    class DataContainer:
        '''A nested dataclass within Exo, reserving space for instantaneous data.'''
        state_time: float = None
        loop_time: float = None
        motor_enc_val: int = 0
        motor_angle: int = 0
        motor_velocity: float = 0
        motor_current: float = 0
        motor_torque_from_current: float = 0
        motor_voltage: float = 0
        cam_enc_val: int = 0
        cam_angle: float = 0
        cable_force_est: float = 0
        ankle_angle: float = 0
        commanded_cam_angle: float = 0
        commanded_cable_force: float = 0
        commanded_position: int = None
        commanded_torque: float = None
        commanded_current: int = None
        temperature: int = None
        control_input: float = None 
        cu_kp: float = None 
        cu_ki: float = None
        cu_kd: float = None
        cam_ang_error: float = None
        cam_ang_error_diff: float = None 
        cam_ang_error_diff_filt: float = None 

    def read_data(self, loop_time=None):
        '''Read data from Dephy Actpack motor, store in actpack_mech.data Data Container.'''
        actpack_data = fxs.read_device(self.dev_id)
        self.last_state_time = self.data.state_time
        self.data.loop_time = loop_time
        self.data.state_time = actpack_data.state_time * self.constants.MS_TO_SECONDS
        self.data.motor_enc_val = actpack_data.mot_ang 
        self.data.motor_current = actpack_data.mot_cur
        self.data.motor_voltage = actpack_data.mot_volt
        self.data.motor_torque_from_current = self._motor_torque_from_current()  
        if actpack_data.ank_ang < self.design_const.CAM_ENC_INIT:
            self.data.cam_enc_val = actpack_data.ank_ang + self.constants.CAM_ENC_PEAK
        else:
            self.data.cam_enc_val = actpack_data.ank_ang
        self.data.temperature = actpack_data.temperature

        if loop_time is not None:
            prev_mot_ang = self.data.motor_angle
            self.data.motor_angle = (self.data.motor_enc_val - self.motor_offset)*self.motor_sign*self.constants.MOT_ENC_CLICKS_TO_DEG        
            temp_cam_ang = abs(self.data.cam_enc_val - self.cam_offset)*self.constants.CAM_ENC_CLICKS_TO_DEG
            if temp_cam_ang<80:
                self.data.cam_angle = temp_cam_ang 
            if self.data.state_time != self.last_state_time and loop_time>1.25/self.constants.ACTPACK_FREQ:
                self.data.motor_velocity = (self.data.motor_angle - prev_mot_ang)/(self.data.state_time - self.last_state_time)            
            self.data.cable_force_est = self._cable_force_est(self.data.cam_angle)
            self.data.ankle_angle = self._compute_ankle_angle()

    def setup_data_writer(self, file_ID: str):
        if file_ID is not None:
            '''file_ID is used as a custom file identifier after date.'''
            # subfolder_name = 'exo_data/'
            self.filename = time.strftime("%Y%m%d_%H%M_") + file_ID + '.csv'
            self.my_file = open(self.filename, 'w', newline='')
            self.writer = csv.DictWriter(
                self.my_file, fieldnames=self.data.__dict__.keys())
            
            self.writer.writeheader()
        
    def write_data(self):
        '''Writes data file, only if new data packet is available'''
        if self.dataFile_name is not None and self.data.state_time!=self.last_state_time:
            self.writer.writerow(self.data.__dict__)
    
    def close_file(self):
        if self.dataFile_name is not None:
            self.my_file.close()


# HELPER FUNCTIONS FOR CONVERSION BETWEEN MOTOR ELECTRICAL VALUES TO MECHANICAL VALUES
    def _cable_force_est(self, cam_ang):
        if cam_ang < self.design_const.CAM_RANGE:
            # USE CAM SPLINE CURVE TO GET DATA
            x = [0, 10, 20, 40, 70, 80, 85]
            y = [2.1622, 2.2054, 2.2703, 2.3784, 6.4865, 9.7297, 10.8108]
            cable_force = pchip_interpolate(x,y,cam_ang)
        else:
            act_torque = self.data.motor_torque_from_current*self.constants.MOTOR_TO_ACTUATOR_TR
            cable_force = act_torque/self.design_const.ACTUATOR_RADIUS
        return cable_force

    def _motor_torque_from_current(self):
        return self.data.motor_current*self.constants.MOTOR_CURRENT_TO_MOTOR_TORQUE
        
    def _unwrapped_cable_length(self):
        # Need to check for sign if unwrapped or actuator anti-clockwise than +ve else -ve
        return self.design_const.ACTUATOR_RADIUS*(self.data.motor_angle)
    
    def _compute_freeEnd_cablelength_change(self):
        free_end_cable_len_change = (self._compute_cam_cablelength(0)-self._compute_cam_cablelength(self.data.cam_angle)) + self._unwrapped_cable_length()
        return free_end_cable_len_change

    def _compute_ankle_angle(self):
        # USE SPLINE CURVE TO RELATE free cable length and ankle angle
        # free_end_cable_len_change = self._compute_freeEnd_cablelength_change()
        # ankle_cable_length = self.design_const.INIT_CABLE_LEN_BW_ANKLE_ACT + free_end_cable_len_change
        ankle_angle = 0
        # ankle_angle = TODO  
        return ankle_angle

    def _compute_cam_cablelength(self, cam_ang):
        '''
        Function computes the Cable length enclosed within the Spring Mechanism.
        Input: CAM Lever Angle with respect to base Angle
        Output: Cable length 
        '''
        r = self.design_const.ROLLER_RADIUS
        l = self.design_const.CAM_LEVER_ARM
        theta_0 = self.design_const.ROLLER_C_INIT_ANG
        cord1 = self.design_const.ROLLER_A_CORD
        cord2 = self.design_const.ROLLER_B_CORD
        cord3 = (l*cos(radians(theta_0+cam_ang)), l*sin(radians(theta_0+cam_ang)))
        cd1 = sqrt((cord3[0]-cord1[0])**2 + (cord3[1]-cord1[1])**2)/2
        cd2 = sqrt((cord3[0]-cord2[0])**2 + (cord3[1]-cord2[1])**2)/2
        cable_length = 2*(sqrt(cd1**2 - r**2) + sqrt(cd2**2 - r**2))
        print(f"cable length for {cam_ang}: {cable_length}")
        return cable_length

    # MOTOR CONTROL COMMANDS
    def update_gains(self, Kp=None, Ki=None, Kd=None, k_val=None, b_val=None, ff=None):
        '''Optionally updates individual gain values, and sends to Actpack motor.'''
        if Kp is not None:
            self.Kp = Kp
        if Ki is not None:
            self.Ki = Ki
        if Kd is not None:
            self.Kd = Kd
        if k_val is not None:
            self.k_val = k_val
        if b_val is not None:
            self.b_val = b_val
        if ff is not None:
            self.ff = ff
        fxs.set_gains(dev_id=self.dev_id, kp=self.Kp, ki=self.Ki,
                      kd=self.Kd, k_val=self.k_val, b_val=self.b_val, ff=self.ff)

    def command_voltage(self, desired_mV: int):
        '''Commands voltage (mV), with positive = PF on right, DF on left.'''
        # Kp = self.constants.default_KP
        # Ki = self.constants.default_KI
        # Kd = self.constants.default_KD
        # k_val = self.constants.default_KVAL
        # b_val = self.constants.default_BVAL
        # ff = self.constants.default_FF
        Kp = 300
        Ki = 300
        Kd = 5
        k_val = 0 
        b_val = 0
        ff = 120    


        if abs(desired_mV) > self.constants.MAX_ALLOWABLE_VOLTAGE_COMMAND:
            print('Commanded higher than allowed volatage, saturated voltage', desired_mV)
            desired_mV = (desired_mV/abs(desired_mV))*self.constants.MAX_ALLOWABLE_VOLTAGE_COMMAND

        self.update_gains(Kp=Kp,Ki=Ki,Kd=Kd,k_val=k_val,b_val=b_val,ff=ff)
        fxs.send_motor_command(dev_id=self.dev_id, ctrl_mode=fxe.FX_VOLTAGE, value=desired_mV)
        self.data.commanded_current = None
        self.data.commanded_position = None
        self.data.commanded_torque = None
    
    def command_current(self, desired_mA: int):
        '''Commands current (mA), with positive = PF on right, DF on left.'''
        Kp = self.constants.default_KP
        Ki = self.constants.default_KI
        Kd = self.constants.default_KD
        k_val = self.constants.default_KVAL
        b_val = self.constants.default_BVAL
        ff = self.constants.default_FF

        if abs(desired_mA) > self.constants.MAX_ALLOWABLE_CURRENT:
            self.command_controller_off()
            raise ValueError('abs(desired_mA) must be < config.max_allowable_current')
        
        self.update_gains(Kp=Kp,Ki=Ki,Kd=Kd,k_val=k_val,b_val=b_val,ff=ff)
        fxs.send_motor_command(dev_id=self.dev_id, ctrl_mode=fxe.FX_CURRENT, value=desired_mA)
        self.data.commanded_current = desired_mA
        self.data.commanded_position = None

    def command_position(self, desired_setpoint):
        '''Commands motor angle (counts). Pay attention to the sign!'''
        fxs.send_motor_command(
            dev_id=self.dev_id, ctrl_mode=fxe.FX_POSITION, value=desired_setpoint)
        self.data.commanded_current = None
        self.data.commanded_position = desired_setpoint
        self.data.commanded_torque = None

    def command_controller_off(self):
        fxs.send_motor_command(dev_id=self.dev_id, ctrl_mode=fxe.FX_NONE, value=0)


    # MECHANISM INITIAL CALIBRATION
    def intial_calibration(self, calibration_mV: int = 2500, max_seconds_to_calibrate: float = 5, current_threshold: float = 1500):
        try:
            '''Brings up slack, calibrates ankle and motor offset angles.'''
            input('Press Enter to calibrate exo on')
            time.sleep(0.2)
            print('Calibrating...')
            self.read_data()
            self.cam_offset = self.data.cam_enc_val
            current_filter = filters.MovingAverage(window_size=10)
            self.command_voltage(desired_mV=self.motor_sign * calibration_mV)
            t0 = time.time()
            while time.time()-t0 < max_seconds_to_calibrate:
                time.sleep(0.01)
                self.read_data()
                # if abs(current_filter.filter(self.data.motor_current)) > current_threshold:
                #     break
                if (abs(self.data.cam_enc_val - self.cam_offset)*self.constants.CAM_ENC_CLICKS_TO_DEG)>60:
                    break
            else:
                # Enters here if while loop doesn't break
                self.command_controller_off()
                raise RuntimeError('Calibration Timed Out!!!')

            self.command_controller_off()
            time.sleep(1)
            i = 0
            temp_cam_ang = 78
            t0 = time.time()
            while temp_cam_ang>5 and time.time()-t0 < 5: 
                time.sleep(0.01)
                self.read_data()
                self.command_voltage(desired_mV = -1 * self.motor_sign * calibration_mV)
                temp_cam_ang = abs(self.data.cam_enc_val - self.cam_offset)*self.constants.CAM_ENC_CLICKS_TO_DEG
                i+=1
  
            if temp_cam_ang>6:
                # Enters here if while loop doesn't break
                self.command_controller_off()
                raise RuntimeError('Calibration Timed Out while unreeling!!!')
            

            self.motor_offset = self.data.motor_enc_val            
            self.command_voltage(desired_mV=0)
            self.read_data()
            while self.data.motor_voltage!=0:
                self.read_data()
                self.command_voltage(desired_mV=0)
                print("Motor_volt: ", self.data.motor_voltage)
                self.command_controller_off()
                time.sleep(0.5)
    
            self.has_calibrated = True
            print("CAM Angle: ", temp_cam_ang)
            print('CAM Offset: ', self.cam_offset)
            print('Motor offset: ', self.motor_offset)        
            print('Finished Calibrating')

        except Exception as err:
            print(traceback.print_exc())
            self.close()
            raise RuntimeError('Calibration Failed')
            return
        

def connect_to_actuator(dataFile_name: str):
    '''Connect to Actuator, instantiate Actuator object'''
    # Connection settings
    DO_DEPHY_LOG = False
    ACTPACK_BAUD_RATE = 52000
    ACTPACK_FREQ = 200
    port = '/dev/ttyACM0'
    try:
        dev_id = fxs.open(port, ACTPACK_BAUD_RATE, log_level=3)
        fxs.start_streaming(
            dev_id=dev_id, freq=ACTPACK_FREQ, log_en=DO_DEPHY_LOG)
    except IOError:
        print('Unable to connect with Actuator on port: ', port)
    actuator = actpack_mech(dev_id, dataFile_name)
    return actuator
