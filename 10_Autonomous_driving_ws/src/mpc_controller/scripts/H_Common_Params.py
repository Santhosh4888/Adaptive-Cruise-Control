# !/usr/bin/env python3

import os
import pickle
import numpy as np
from scipy.interpolate import Rbf

main_internal_scripts_dir = os.path.dirname(os.path.abspath(__file__))
main_internal_MPC_dir = os.path.abspath(os.path.join(main_internal_scripts_dir, '..'))
main_internal_data_dir = os.path.abspath(os.path.join(main_internal_MPC_dir, 'data'))

# The parameters for the PD Controller

Dd = 1.0                                                                           # Desired Seperation in [m]
Td = 2.0                                                                           # Time delay in [s]
Tbl = 3.0                                                                          # Time for which the controller won't respond in [s]
Vr = 0.0                                                                           # Desired Relative Velocity [m/s]
a_max = 1.4                                                                        # Maximum acceleration [m/s^2]
a_min = -3.5                                                                       # Minimum deceleration [m/s^2]
acc_Jerk_limit = 1.5                                                               # Jerk limit for acceleration [m/s^3]
dec_Jerk_limit = -3.0                                                              # Jerk limit for deceleration [m/s^3] 

GVW = 680                                                                          # Gross vehicle weight of vehicle is 680 kg
VW = 1.5 * 680                                                                     # Adding a factor of safety of 1.5 
MAX_motor_torque = 16                                                              # Nm
MAX_motor_power = 7500                                                             # Watts
te = 0.9                                                                           # Transmission efficiency

sample_time = 0.1                                                                  # Sample time of the Radar sensor
periodic_step = 0.1                                                                # Periodic step in seconds same as sample time
H_periodic_step = 0.5                                                              # Periodic step for hardware

Dr_1 = 10.0                                                                        # Radar measured Distance (Initial Seperation) (Scenario 1 : cut-in scenario)
Dr_2 = 100.0                                                                       # Radar measured Distance (Initial Seperation) (Scenario 2 : Vehicle in front moving at a low speed)
Dr_3 = 100.0                                                                       # Radar measured Distance (Initial Seperation) (Scenario 3 : coming to a complete stop)
Dr_4 = 15.0                                                                        # Radar measured Distance (Initial Seperation) (Scenario 4 : cut-in scenario)

total_experiment_time = 150.0                                                      # Total experiment time in [s]
H_total_experiment_time = 50.0                                                     # Total experiment time in [s] for Hardware

Vp_1 = 16.0 * (5 / 18)                                                             # Preceeding Car Velocity in [km/hr * (5/18) = m/s]
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

linear_params_file_path = os.path.join(main_internal_data_dir, 'linear_params.pkl')
A = None
with open(linear_params_file_path, 'rb') as f:
    A = pickle.load(f)[0]                                                          # Constant for the Linear model

slope_rpm_tcmd_file_path = os.path.join(main_internal_data_dir, 'slope_rpm_tcmd.pkl')
slope_rpm_tcmd = None
with open(slope_rpm_tcmd_file_path, 'rb') as f:
    slope_rpm_tcmd = pickle.load(f)[0]                                             # The slope of the line relating Motor rpm to throttle command 

velocity_to_rpm_ratio = 60 * gear_ratio / (2 * np.pi * tyre_radius)                # Velocity to RPM ratio

safety_factor = 0.9
i_safety_factor = 1.1
    
threshold_throttle_cmd = 5.0                                                       # Below this throttle command, set throttle command as 0
threshold_velocity = max(0, min(max_rpm, slope_rpm_tcmd * (threshold_throttle_cmd - 0))) / velocity_to_rpm_ratio

rbf_model_tcmd_to_requested_pot_file_path = os.path.join(main_internal_data_dir, 'rbf_model_tcmd_to_requested_pot.pkl')
rbf_model_tcmd_to_requested_pot, rbf_model_tcmd_to_requested_pot_data = None, None
with open(rbf_model_tcmd_to_requested_pot_file_path, 'rb') as f:
    rbf_model_tcmd_to_requested_pot_data = pickle.load(f)                          # This stores the rbf weights to convert tcmd to requested throttle pot
rbf_model_tcmd_to_requested_pot = Rbf(rbf_model_tcmd_to_requested_pot_data['x'], rbf_model_tcmd_to_requested_pot_data['y'], function = rbf_model_tcmd_to_requested_pot_data['function'])