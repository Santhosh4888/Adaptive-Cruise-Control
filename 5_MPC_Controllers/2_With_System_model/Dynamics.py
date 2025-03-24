import sys
CP_module_dir = r'D:\lord_of_darkness\IITM\5th_year\DDP_Final\6_Common_Paramaters'
sys.path.append(CP_module_dir)
import Common_Params as CP
import numpy as np
import System_Model as SM

class kinematics_1:

    def __init__(self):

        self.cur_position = 0.0
        self.cur_acceleration = 0.0
        self.sys_model = SM.System()
        self.sys_model.load_state_dict(CP.non_linear_weights)
        self.sys_model.reset_seq()

    def update_velocity(self, cur_V):

        self.cur_velocity = cur_V
        return [self.cur_position, self.cur_velocity]
    
    def rpm_to_velocity(self, rpm):                                                    # RPM should be in rpm
        return rpm * 2 * np.pi * CP.tyre_radius / (CP.gear_ratio * 60)

    def velocity_to_rpm(self, velocity):                                               # Velocity should be in m/s
        return velocity * 60 * CP.gear_ratio / (2 * np.pi * CP.tyre_radius)
    
    def get_motor_rpm(self, throttle_cmd):
        return max(0, min(CP.max_rpm, CP.slope_rpm_tcmd * (throttle_cmd - 0)))

    def get_throttle_cmd(self, motor_rpm):
        return max(0, min(CP.max_tcmd, 1 / CP.slope_rpm_tcmd * (motor_rpm - 0)))

    def control(self, a):

        cur_rpm = self.velocity_to_rpm(self.cur_velocity)
        
        req_vel = self.cur_velocity + a / CP.A
        req_vel = max(0.0, min(req_vel, CP.ego_max_v))
        throttle_cmd = self.get_throttle_cmd(self.velocity_to_rpm(req_vel))
        if throttle_cmd <= CP.threshold_throttle_cmd:
            throttle_cmd = 0.0
        
        self.sys_model.get_nn_input(throttle_cmd, cur_rpm)                                    # Converts the current motor rpm and throttle command into a form suitable for neural network
        prediction = self.sys_model(self.sys_model.input_seq)                                 # Gets the prediction of neural network
        fin_rpm = CP.scaler.inverse_transform([[0, 0, 0, 0, 0, 0, prediction.item()]])[0, -1] # Descales the prediction of the neural network and gets the final output rpm
        
        self.cur_velocity = self.rpm_to_velocity(fin_rpm)
        self.cur_velocity = max(min(CP.ego_max_v, self.cur_velocity), 0.0)

        self.cur_position += self.cur_velocity * CP.sample_time
        self.cur_acceleration = a
    
        return [self.cur_position, self.cur_velocity]


class kinematics_2:

    def __init__(self, Dr):

        self.cur_position = Dr
    
    def update_velocity(self, cur_V):
        
        self.cur_velocity = cur_V
        return [self.cur_position, self.cur_velocity]
    
    def control(self):

        self.cur_position += self.cur_velocity * CP.sample_time
        return [self.cur_position, self.cur_velocity]
    
if __name__ == '__main__':
    k1 = kinematics_1()