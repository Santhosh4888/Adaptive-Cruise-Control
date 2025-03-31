import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import Controller
internal_HMPC_dir = os.path.dirname(os.path.abspath(__file__))
internal_HT_dir = os.path.abspath(os.path.join(internal_HMPC_dir, '..'))
internal_H_dir = os.path.abspath(os.path.join(internal_HT_dir, '..'))
root_dir = os.path.abspath(os.path.join(internal_H_dir, '..'))
internal_CP_dir = os.path.abspath(os.path.join(root_dir, '6_Common_Paramaters'))
sys.path.append(internal_CP_dir)
import Common_Params as CP

class VLC:                                                                             # Vehicle Longitudinal Controller
    
    def __init__(self):
        
        self.controller = Controller.MPC()                                             # Defining the controller
        self.throttle_pot = 0.0                                                        # Storing the initial control action
        
    def rpm_to_velocity(self, rpm):                                                    # RPM should be in rpm
        
        return rpm * 2 * np.pi * CP.tyre_radius / (CP.gear_ratio * 60)

    def velocity_to_rpm(self, velocity):                                               # Velocity should be in m/s
        
        return velocity * 60 * CP.gear_ratio / (2 * np.pi * CP.tyre_radius)
    
    def get_motor_rpm(self, throttle_cmd):
        
        return max(0, min(CP.max_rpm, CP.slope_rpm_tcmd * (throttle_cmd - 0)))

    def get_throttle_cmd(self, motor_rpm):
        
        return max(0, min(CP.max_tcmd, 1 / CP.slope_rpm_tcmd * (motor_rpm - 0)))
    
    def get_control_action(self, ego_values, preceeding_values):                                 # ego_values = [ego_pos, ego_vel], preceeding_values = [obs_pos, obs_vel]
        
        a = self.controller.get_acceleration(ego_values, preceeding_values)                      # Getting control action from the controller
        req_vel = max(0.0, min(CP.ego_max_v, self.ego_values[1] + a / CP.A))                     # Calculating required velocity from the acceleration
        throttle_cmd = self.get_throttle_cmd(self.velocity_to_rpm(req_vel))                      # Calculating throttle command from required velocity
        if throttle_cmd <= CP.threshold_throttle_cmd:                                            # Restricting throttle command below 5 percent to zero
            throttle_cmd = 0.0
        self.throttle_pot = CP.rbf_model_tcmd_to_requested_pot(np.array([throttle_cmd]))[0]      # Calculating the required throttle pot from throttle command               
        
        
        
        
        

