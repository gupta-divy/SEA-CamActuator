from ACTUATOR_CODE import actpack_mech
from math import pi
import filters

class Controller(object):
    '''Parent controller object. Child classes inherit methods.'''

    def __init__(self, mechanism: actpack_mech):
        self.mechanism = mechanism

    def command(self, reset):
        '''For modularity, new controllers will ideally not take any arguments with
        their command() function. The exo object stored on self will have updated
        data, which is accessible to controller objects.'''
        raise ValueError('command() not defined in child class of Controller')

    def update_controller_gains(self, Kp: int, Ki: int, Kd: int = 0, ff: int = 0):
        '''Updated internal controller gains. Note: None (default) means no change will be commanded.'''
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.ff = ff

    def command_gains(self):
        self.mechanism.update_gains(Kp=self.Kp, Ki=self.Ki, Kd=self.Kd, ff=self.ff)

    # def update_ctrl_params_from_config(self, config: Type[config_util.ConfigurableConstants]):
    #     '''For modularity, new controllers ideally use this function to update internal
    #     control params (e.g., k_val, or rise_time) from the config object. If needed, add
    #     new ctrl params to ConfigurableConstants.'''
    #     raise ValueError(
    #         'update_ctrl_params_from_config() not defined in child class of Controller')

class SpringMechController(Controller):
    def __init__(self, mechanism: actpack_mech, desired_cam_ang, cam_Kp:float, cam_Ki:float, cam_Kd:float, Kp:int, Kd:int, Ki:int, ff:int, saturation_value: int):
        self.mechanism = mechanism
        self.ang_prev_error = 0
        self.ang_integral_error = 0
        self.cam_Kp = cam_Kp
        self.cam_Ki = cam_Ki
        self.cam_Kd = cam_Kd
        self.desired_cam_angle = desired_cam_ang
        self.saturation_value = saturation_value
        super().update_controller_gains(Kp=Kp,Kd=Kd,Ki=Ki,ff=ff)
        self.butterfilter = filters.Butterworth(N=2,Wn=20,fs=200)

    def command(self, reset=False):
        if reset:
            self.ang_integral_error = 0
            self.ang_prev_error = 0
            super().command_gains()
            self.mechanism.data.commanded_cam_angle = self.desired_cam_angle
            self.mechanism.data.commanded_cable_force  = self.mechanism._cable_force_est(self.desired_cam_angle)
        
        desired_motor_setpoint = self.compute_control()
        self.mechanism.command_position(desired_motor_setpoint)

    def compute_control(self):
        required_motor_ang_compensation = self.get_motor_ang_comp_for_external_cable_move()
        act_required_mot_ang_comp = self.get_motor_ang_comp_for_des_cam_ang()
        print("mot_ang_comp from calc: ", act_required_mot_ang_comp)
        print("mot_ang_comp from PID: ", required_motor_ang_compensation)

        #Saturation on velocity value:
        if abs(required_motor_ang_compensation)>self.saturation_value:
            print("Motor Angle compensation for CAM too High, saturating it to 360")
            required_motor_ang_compensation = self.saturation_value *  (required_motor_ang_compensation / abs(required_motor_ang_compensation))
        self.mechanism.data.control_input = 1*required_motor_ang_compensation + 0*act_required_mot_ang_comp
        required_motor_encVal_change = (required_motor_ang_compensation / self.mechanism.constants.MOT_ENC_CLICKS_TO_DEG)*self.mechanism.constants.MOTOR_TO_ACTUATOR_TR
        motor_setpoint = self.mechanism.data.motor_enc_val + required_motor_encVal_change*self.mechanism.motor_sign
        return motor_setpoint
    
    def get_motor_ang_comp_for_external_cable_move(self):
        ang_error = self.desired_cam_angle - self.mechanism.data.cam_angle
        ang_error_integ = self.ang_integral_error + ang_error
        dt = (self.mechanism.data.state_time - self.mechanism.last_state_time)
        
        if dt!=0:
            ang_error_diff = (ang_error - self.ang_prev_error) / dt
        else:
            ang_error_diff = (ang_error - self.ang_prev_error) / 0.005
        self.mechanism.data.cam_ang_error = ang_error
        self.mechanism.data.cam_ang_error_diff = ang_error_diff
        ang_error_diff = self.butterfilter.filter(ang_error_diff)
        self.mechanism.data.cam_ang_error_diff_filt = ang_error_diff
        self.mechanism.data.cu_kp = self.cam_Kp*ang_error
        self.mechanism.data.cu_ki = self.cam_Ki*ang_error_integ
        self.mechanism.data.cu_kd = self.cam_Kd*ang_error_diff
        motor_ang_compensation = self.cam_Kp*ang_error + self.cam_Kd*ang_error_diff + self.cam_Ki*ang_error_integ
        self.ang_prev_error = ang_error
        self.ang_integral_error = ang_error_integ
        print("CAM Angle error: ", ang_error)
        print("CAM Angle error diff: ", ang_error_diff)
        return motor_ang_compensation

# not used for control here
    def get_motor_ang_comp_for_des_cam_ang(self):
        cable_length_error = self.mechanism._compute_cam_cablelength(self.mechanism.data.cam_angle) - self.mechanism._compute_cam_cablelength(self.desired_cam_angle)
        motor_ang_compensation = cable_length_error / self.mechanism.design_const.ACTUATOR_RADIUS
        motor_ang_compensation = motor_ang_compensation*180/pi
        return motor_ang_compensation
