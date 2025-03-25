import os
import pickle
import numpy as np

# The parameters for the PD Controller

Dd = 1.0                                                                           # Desired Seperation in [m]
Td = 2.0                                                                           # Time delay in [s]
Tbl = 3.0                                                                          # Time for which the controller won't respond in [s]
Vr = 0.0                                                                           # Desired Relative Velocity [m/s]
a_max = 1.4                                                                        # Maximum acceleration [m/s^2]
a_min = -3.5                                                                       # Minimum deceleration [m/s^2]
acc_Jerk_limit = 3.0                                                               # Jerk limit for acceleration [m/s^3]
dec_Jerk_limit = -3.0                                                              # Jerk limit for deceleration [m/s^3] 

GVW = 680                                                                          # Gross vehicle weight of vehicle is 680 kg
VW = 1.5 * 680                                                                     # Adding a factor of safety of 1.5 
MAX_motor_torque = 16                                                              # Nm
MAX_motor_power = 7500                                                             # Watts
te = 0.9                                                                           # Transmission efficiency

sample_time = 0.1                                                                  # Sample time of the Radar sensor
periodic_step = 0.1                                                                # Periodic step in seconds same as sample time
Dr_1 = 10.0                                                                        # Radar measured Distance (Initial Seperation) (Scenario 1 : cut-in scenario)
Dr_2 = 100.0                                                                       # Radar measured Distance (Initial Seperation) (Scenario 2 : Vehicle in front moving at a low speed)
Dr_3 = 100.0                                                                       # Radar measured Distance (Initial Seperation) (Scenario 3 : coming to a complete stop)
Dr_4 = 15.0                                                                        # Radar measured Distance (Initial Seperation) (Scenario 4 : cut-in scenario)

total_experiment_time = 150.0                                                      # Total experiment time in [s]

Vp_1 = 18.0 * (5 / 18)                                                             # Preceeding Car Velocity in [km/hr * (5/18) = m/s]
Vp_2 = 12.0 * (5 / 18)                                                             # Preceeding Car Velocity in [km/hr * (5/18) = m/s]
Vp_3 = 0.0 * (5 / 18)                                                              # Preceeding Car Velocity in [km/hr * (5/18) = m/s]
Vp_4 = 12.0 * (5 / 18)                                                             # Preceeding Car Velocity in [km/hr * (5/18) = m/s]      # Minimum possible speed is 17 km/hr

Drs = [Dr_1, Dr_2, Dr_3, Dr_4]
Vps = [Vp_1, Vp_2, Vp_3, Vp_4]
 
start_ego_values = 0.0 * (5 / 18)                                                  # Velocities of ego vehicle in [km/hr * (5/18) = m/s]
ego_max_v = 20.0 * (5 / 18)
max_tcmd = 100.0


gear_ratio = 12.0                                                                  # Gear ratio of the vehicle
tyre_radius = 0.265                                                                # Tyre radius of the vehicle

# Indirect variables

max_rpm = ego_max_v * 60 * gear_ratio / (2 * np.pi * tyre_radius)                  # Maximum rpm of ego vehicle in RPM

linear_params_folder_path = r'D:\lord_of_darkness\IITM\5th_year\DDP_Final\3_System_Identification\Linear_System_Model'
linear_params_file_path = os.path.join(linear_params_folder_path, 'linear_params.pkl')
A = None
with open(linear_params_file_path, 'rb') as f:
    A = pickle.load(f)[0]                                                          # Constant for the Linear model

non_linear_params_folder_path = r'D:\lord_of_darkness\IITM\5th_year\DDP_Final\3_System_Identification\Non_Linear_System_Model'
non_linear_params_file_path = os.path.join(non_linear_params_folder_path, 'non_linear_params.pkl')
non_linear_weights = None
with open(non_linear_params_file_path, 'rb') as f:
    non_linear_weights = pickle.load(f)                                            # Non linear model weights

slope_rpm_tcmd_folder_path = r'D:\lord_of_darkness\IITM\5th_year\DDP_Final\2_Data_Analysis'
slope_rpm_tcmd_file_path = os.path.join(slope_rpm_tcmd_folder_path, 'slope_rpm_tcmd.pkl')
slope_rpm_tcmd = None
with open(slope_rpm_tcmd_file_path, 'rb') as f:
    slope_rpm_tcmd = pickle.load(f)[0]                                             # The slope of the line relating Motor rpm to throttle command
    
scaler_folder_path = r'D:\lord_of_darkness\IITM\5th_year\DDP_Final\3_System_Identification\Non_Linear_System_Model'
scaler_file_path = os.path.join(scaler_folder_path, 'scaler.pkl')
scaler = None
with open(scaler_file_path, 'rb') as f:
    scaler = pickle.load(f)                                                        # Scaler for storing MINMAXSCALER values 
    
neural_network_params_folder_path = r'D:\lord_of_darkness\IITM\5th_year\DDP_Final\3_System_Identification\Non_Linear_System_Model'
neural_network_params_file_path = os.path.join(neural_network_params_folder_path, 'Neural_network_params.pkl')
neural_network_params = None
with open(neural_network_params_file_path, 'rb') as f:
    neural_network_params = pickle.load(f)                                         # Params of neural network
    
seq_length, batch_size, num_epochs, learning_rate, hidden_size_1, hidden_size_2, hidden_size_3, hidden_size_4, num_layers_1, num_layers_2, input_size, output_size = neural_network_params

velocity_to_rpm_ratio = 60 * gear_ratio / (2 * np.pi * tyre_radius)                # Velocity to RPM ratio

safety_factor = 0.9
i_safety_factor = 1.1

stopping_velocities_distances_folder_path = r'D:\lord_of_darkness\IITM\5th_year\DDP_Final\2_Data_Analysis'
stopping_velocities_distances_file_path = os.path.join(stopping_velocities_distances_folder_path, 'stopping_velocities_distances.pkl')
stopping_velocities_distances_rbf = None
with open(stopping_velocities_distances_file_path, 'rb') as f:
    stopping_velocities_distances_rbf = pickle.load(f)
    
threshold_throttle_cmd = 5.0                                                       # Below this throttle command, set throttle command as 0