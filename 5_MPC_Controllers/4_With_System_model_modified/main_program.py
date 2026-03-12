import numpy as np
import matplotlib.pyplot as plt
import Controller
import Dynamics
import sys
import os
internal_WSM_dir = os.path.dirname(os.path.abspath(__file__))
internal_MPC_dir = os.path.abspath(os.path.join(internal_WSM_dir, '..'))
root_dir = os.path.abspath(os.path.join(internal_MPC_dir, '..'))
internal_CP_dir = os.path.abspath(os.path.join(root_dir, '6_Common_Paramaters'))
sys.path.append(internal_CP_dir)
import Common_Params as CP

def main():
    store_seperations = [[] for _ in range(len(CP.Drs))]
    store_acceleration = [[] for _ in range(len(CP.Drs))]
    store_velocities = [[] for _ in range(len(CP.Drs))]
    store_time = [[] for _ in range(len(CP.Drs))]
    store_detected_times = [None for _ in range(len(CP.Drs))]
    store_start_reaction_times = [None for _ in range(len(CP.Drs))]

    for i in range(len(CP.Drs)):
        upper_controller = Controller.MPC()   # now uses Np=20, Nc=10

        if i == 0:
            upper_controller.desired_speed = 20 * 5 / 18   # 20 km/h for scenario 1
        else:
            upper_controller.desired_speed = CP.ego_max_v

        ego_value = CP.start_ego_values
        ego = Dynamics.kinematics_1()

        # Initial offset for preceding vehicle
        if i == 0:
            pre = Dynamics.kinematics_2(CP.Drs[i] + 10)
        else:
            pre = Dynamics.kinematics_2(CP.Drs[i] + 25)

        ego_values = ego.update_velocity(ego_value)
        preceeding_values = pre.update_velocity(CP.Vps[i])

        time = 0.0
        detected = False                # True once obstacle is within initial distance

        while time < CP.total_experiment_time:
            # Store data (velocity in km/h, separation in m)
            store_velocities[i].append(ego_values[1] * 18 / 5)
            store_seperations[i].append(preceeding_values[0] - ego_values[0])
            store_time[i].append(time)

            # Detection logic: once separation <= initial distance, obstacle is considered
            if not detected and (preceeding_values[0] - ego_values[0]) <= CP.Drs[i]:
                detected = True
                store_detected_times[i] = time
                # Reaction is immediate; you could add a delay here if desired

            # Provide obstacle to controller only if detected
            if detected:
                obs_values = preceeding_values
            else:
                obs_values = None

            a = upper_controller.get_acceleration(ego_values=ego_values,
                                                   preceeding_values=obs_values)
            store_acceleration[i].append(a)

            print(f'Separation : {store_seperations[i][-1]}, Velocity : {store_velocities[i][-1]}, Acceleration : {store_acceleration[i][-1]}')

            # Always use the same time step (no jump when velocity low)
            dt = CP.sample_time

            ego_values = ego.control(a)
            preceeding_values = pre.control(dt)

            time += dt

            # If you want to record the moment the vehicle starts reacting,
            # set it to the detection time (or add a fixed delay)
            if detected and store_start_reaction_times[i] is None:
                store_start_reaction_times[i] = time

    return [store_seperations, store_acceleration, store_velocities, store_time,
            store_detected_times, store_start_reaction_times]

if __name__ == '__main__':
    results = main()
    print("Program ended")