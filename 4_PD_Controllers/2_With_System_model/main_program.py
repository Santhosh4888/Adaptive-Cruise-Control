import numpy as np
import matplotlib.pyplot as plt
import Controller
import Dynamics
import sys
import os
internal_WSM_dir = os.path.dirname(os.path.abspath(__file__))
internal_PDC_dir = os.path.abspath(os.path.join(internal_WSM_dir, '..'))
root_dir = os.path.abspath(os.path.join(internal_PDC_dir, '..'))
internal_CP_dir = os.path.abspath(os.path.join(root_dir, '6_Common_Paramaters'))
sys.path.append(internal_CP_dir)
import Common_Params as CP

def main(kp):

    store_seperations = [[] for _ in range(len(CP.Drs))]                   # For storing the seperations for different cases
    store_acceleration = [[] for _ in range(len(CP.Drs))]                  # For storing the acceleration input for different cases
    store_velocities = [[] for _ in range(len(CP.Drs))]                    # For storing the velocities of ego vehicle for different cases
    store_time = [[] for _ in range(len(CP.Drs))]                          # For storing the time taken for plotting purposes
    
    store_detected_times = [None for _ in range(len(CP.Drs))]              # Stores the time at which the obstacle is being detected
    store_start_reaction_times = [None for _ in range(len(CP.Drs))]        # Stores the time at which the vehicle starts to react
    
    for i in range(len(CP.Drs)):
        upper_controller = Controller.PID_Controller()                     # The Upper Controller
        upper_controller.set_proportional(kp)                              # Set proportional and derivative contoller constants        
        
        if i == 0:
            upper_controller.desired_speed = 20 * 5 / 18                   # Desired velocity of ego vehicle in [km/hr * (5/18) = m/s]
        else:
            upper_controller.desired_speed = CP.ego_max_v

        ego_value = CP.start_ego_values
        ego = Dynamics.kinematics_1()                                      # For setting the kinematics equations of the ego vehicle 
        if i == 0:
            pre = Dynamics.kinematics_2(CP.Drs[i] + 10)                    # For setting the kinematics equations of the preceeding vehicle
        else:
            pre = Dynamics.kinematics_2(CP.Drs[i] + 25)                    # For setting the kinematics equations of the preceeding vehicle

        ego_values = ego.update_velocity(ego_value)
        preceeding_values = pre.update_velocity(CP.Vps[i])

        time = 0.0                                                         # Start time 0.0 seconds
        detected_time = 0.0                                                # Time for storing the detected time
        detected = 'Not detected'                                          # Boolean to store wether the vehicle has detected the obstacle or not
        while time < CP.total_experiment_time:

            store_velocities[i].append(ego_values[1] * 18 / 5)                                                    # Stores ego vehicle velocities in km/hr
            store_seperations[i].append(preceeding_values[0] - ego_values[0])                                     # Stores the current seperation
            store_time[i].append(time)    

            if preceeding_values[0] - ego_values[0] > CP.Drs[i] and detected == 'Not detected':
                preceeding_values = None
                
            elif detected == 'Not detected':
                preceeding_values = None
                detected = 'first'
                store_detected_times[i] = time
            
            elif detected == 'first':
                preceeding_values = None
            
            a = upper_controller.get_acceleration(ego_values = ego_values, preceeding_values = preceeding_values) # Get the control input
            
            store_acceleration[i].append(a)                                                                       # Stores the acceleration values                                                                         # Store the time

            ego_values = ego.control(a)
            preceeding_values = pre.control()
            
            time += CP.sample_time
            
            if detected == 'first':
                detected_time += CP.sample_time
            else:
                detected_time = 0
            
            if detected_time > CP.Tbl and detected == 'first':
                detected = 'second'
                store_start_reaction_times[i] = time
    
    return [store_seperations, store_acceleration, store_velocities, store_time, store_detected_times, store_start_reaction_times]
            
if __name__ == '__main__':
    store_seperations, store_acceleration, store_velocities, store_time = main(0.0001)
    print("Program ended")