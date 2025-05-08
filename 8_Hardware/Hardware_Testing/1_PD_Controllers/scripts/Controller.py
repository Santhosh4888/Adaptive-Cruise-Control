#!/usr/bin/env python3

import numpy as np
import H_Common_Params as CP

class PID_Controller:                                                  
    
    def __init__(self, kp = 0.0001, ki = 0.0):
        
        self.kp = kp
        self.kd = 2 * np.sqrt(kp)                                             # Condition for critical damping case
        self.ki = ki
        self.a_cur = 0                                                        # Current acceleration 
        self.desired_speed = CP.ego_max_v
    
    def set_proportional(self, kp):
        self.kp = kp
        self.set_derivative()
    
    def set_derivative(self):
        #self.kd = 2 * np.sqrt(self.kp)
        self.kd = 2 * np.sqrt((self.kp * CP.Td) ** 2 + self.kp) - 2 * self.kp * CP.Td
    
    def set_integral(self, ki):
        self.ki = ki
    
    def get_acceleration(self, ego_values, preceeding_values):               # ego_values = [ego_position, ego_velocity]
                                                                             # preceeding_values = [preceeding_vehicle_position, preceeding_vehicle_velocity] 
        if preceeding_values is None:
            a = self.kd * (self.desired_speed - ego_values[1])
        else:
            a = self.kp * ((preceeding_values[0] - (CP.Dd + CP.Td * ego_values[1])) - ego_values[0])
            a += self.kd * (preceeding_values[1] - ego_values[1])
        a = max(self.a_cur + CP.dec_Jerk_limit * CP.sample_time, min(self.a_cur + CP.acc_Jerk_limit * CP.sample_time, a))
        self.a_cur = max(CP.a_min, min(a, CP.a_max))   
        return self.a_cur                        
    
    
    
