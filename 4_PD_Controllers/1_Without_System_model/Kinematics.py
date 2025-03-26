import sys
import os
internal_WSM_dir = os.path.dirname(os.path.abspath(__file__))
internal_PDC_dir = os.path.abspath(os.path.join(internal_WSM_dir, '..'))
root_dir = os.path.abspath(os.path.join(internal_PDC_dir, '..'))
internal_CP_dir = os.path.abspath(os.path.join(root_dir, '6_Common_Paramaters'))
sys.path.append(internal_CP_dir)
import Common_Params as CP

class kinematics_1:

    def __init__(self):
        
        self.cur_position = 0.0
        self.cur_acceleration = 0.0
    
    def update_velocity(self, cur_V):

        self.cur_velocity = cur_V
        return [self.cur_position, self.cur_velocity]

    def control(self, a):

        self.cur_position += self.cur_velocity * CP.sample_time + 0.5 * a * CP.sample_time ** 2
        self.cur_velocity += a * CP.sample_time
        self.cur_acceleration = a
        self.cur_velocity = max(min(CP.ego_max_v, self.cur_velocity), 0.0)
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