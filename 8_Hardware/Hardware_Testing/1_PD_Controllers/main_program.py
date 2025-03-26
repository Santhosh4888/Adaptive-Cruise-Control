import numpy as np
import matplotlib.pyplot as plt
import Controller
import sys
import os
internal_HPDC_dir = os.path.dirname(os.path.abspath(__file__))
internal_HT_dir = os.path.abspath(os.path.join(internal_HPDC_dir, '..'))
internal_H_dir = os.path.abspath(os.path.join(internal_HT_dir, '..'))
root_dir = os.path.abspath(os.path.join(internal_H_dir, '..'))
internal_CP_dir = os.path.abspath(os.path.join(root_dir, '6_Common_Paramaters'))
sys.path.append(internal_CP_dir)
import Common_Params as CP

def main(kp):

    store_seperations = []                                                 # For storing the seperations for different cases
    store_acceleration = []                                                # For storing the acceleration input for different cases
    store_velocities = []                                                  # For storing the velocities of ego vehicle for different cases
    store_time = []                                                        # For storing the time taken for plotting purposes
    
    upper_controller = Controller.PID_Controller()                         # The Upper Controller
    upper_controller.set_proportional(kp)                                  # Set proportional and derivative contoller constants        
        
    upper_controller.desired_speed = 20 * 5 / 18                           # Desired velocity of ego vehicle in [km/hr * (5/18) = m/s]

    ego_value = None                         # Get from the controller, need to write code

    ego_values = None
    preceeding_values = None

    time = 0.0                                                             # Start time 0.0 seconds
    detected = 'Not detected'                                          # Boolean to store wether the vehicle has detected the obstacle or not
    while time < CP.total_experiment_time:

        store_velocities.append(ego_values[1] * 18 / 5)                                                    # Stores ego vehicle velocities in km/hr
        store_seperations.append(preceeding_values[0] - ego_values[0])                                     # Stores the current seperation
        store_time.append(time)    

        if preceeding_values[0] - ego_values[0] > 100.0 and detected == 'Not detected':
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