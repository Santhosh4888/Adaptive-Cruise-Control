import json
import uuid
import os

# Load the FV notebook
with open('PDC_VS_MPC_Comparison_FV.ipynb', 'r') as f:
    nb_fv = json.load(f)

# Create markdown header cell
cell_energy_header = {
    "cell_type": "markdown",
    "id": str(uuid.uuid4()),
    "metadata": {},
    "source": ["### Energy Consumption Calculations (FV Scenario)"]
}

# Create energy calculation function cell
cell_energy_function = {
    "cell_type": "code",
    "id": str(uuid.uuid4()),
    "metadata": {},
    "source": [
"""import numpy as np

def calculate_energy_consumption(df, mass_kg=680, mu_rolling=0.015, g=9.81, time_limit_s=50):
    '''Calculate energy consumption for first 50 seconds (1 Hz resampled sampling).'''

    # Filter data to first 50 seconds only
    df_filtered = df[df['Time(s)'] <= time_limit_s].copy()

    if len(df_filtered) < 2:
        print(f"  [WARNING] Not enough data within {time_limit_s}s limit")
        return None

    # Resample to uniform 1 Hz frequency for fair comparison
    time_orig = df_filtered['Time(s)'].values
    time_resampled = np.arange(0, time_limit_s + 1, 1)
    time_resampled = time_resampled[time_resampled <= time_limit_s]

    # Interpolate acceleration and velocity to common time grid
    accel_orig = df_filtered['Ego_Acceleration(m/s^2)'].values
    accel_resampled = np.interp(time_resampled, time_orig, accel_orig)

    velocity_kmph_orig = df_filtered['Ego_Velocity_kmph'].values
    velocity_kmph_resampled = np.interp(time_resampled, time_orig, velocity_kmph_orig)
    velocity_resampled = velocity_kmph_resampled / 3.6  # Convert to m/s

    time = time_resampled
    accel = accel_resampled
    velocity = velocity_resampled

    # Calculate time intervals
    dt = np.diff(time)

    # Rolling friction force (constant)
    F_friction = mu_rolling * mass_kg * g

    # Acceleration force
    F_accel = mass_kg * accel

    # Total driving force
    F_total = F_accel + F_friction

    # Power = Force x Velocity (Watts)
    power = F_total * velocity
    power = np.maximum(power, 0)

    # Energy = integral Power dt using trapezoidal rule
    power_intervals = (power[:-1] + power[1:]) / 2
    energy_interval = power_intervals * dt
    total_energy_j = np.sum(energy_interval)

    # Breakdown: Acceleration energy
    power_accel = mass_kg * accel * velocity
    power_accel = np.maximum(power_accel, 0)
    power_accel_intervals = (power_accel[:-1] + power_accel[1:]) / 2
    energy_accel_j = np.sum(power_accel_intervals * dt)

    # Breakdown: Friction energy
    power_friction = F_friction * velocity
    power_friction_intervals = (power_friction[:-1] + power_friction[1:]) / 2
    energy_friction_j = np.sum(power_friction_intervals * dt)

    # Statistics
    avg_power = total_energy_j / (time[-1] - time[0]) if len(time) > 1 else 0
    peak_power = np.max(power)

    return {
        'total_energy_j': total_energy_j,
        'total_energy_kj': total_energy_j / 1000,
        'total_energy_kwh': total_energy_j / 3.6e6,
        'energy_accel_kj': energy_accel_j / 1000,
        'energy_friction_kj': energy_friction_j / 1000,
        'avg_power_w': avg_power,
        'peak_power_w': peak_power,
        'time_duration_s': time[-1] - time[0],
        'actual_samples': len(time),
        'sampling_freq_hz': 1.0,
    }

print("Energy consumption function defined (1 Hz resampled)")
"""
    ]
}

# Create FV 5kph energy analysis cell
cell_energy_5kph_fv = {
    "cell_type": "code",
    "id": str(uuid.uuid4()),
    "metadata": {},
    "source": [
"""# ENERGY CONSUMPTION - 5 KPH FOLLOWING VEHICLE SCENARIO

import re

# Get 5 kph files from both controllers
pdc_5kph_fv_files = [f for f in os.listdir(internal_PDC_FV_data_dir) if '5kph' in f]
mpc_5kph_fv_files = [f for f in os.listdir(internal_MPC_FV_data_dir_1) if '5kph' in f]

print(f"PD 5kph FV files: {pdc_5kph_fv_files}")
print(f"MPC 5kph FV files: {mpc_5kph_fv_files}")

if pdc_5kph_fv_files and mpc_5kph_fv_files:
    all_energy_results_5kph = []

    for pdc_file in pdc_5kph_fv_files:
        for mpc_file in mpc_5kph_fv_files:
            pdc_df = pd.read_csv(os.path.join(internal_PDC_FV_data_dir, pdc_file))
            mpc_df = pd.read_csv(os.path.join(internal_MPC_FV_data_dir_1, mpc_file))

            pdc_energy = calculate_energy_consumption(pdc_df, time_limit_s=50)
            mpc_energy = calculate_energy_consumption(mpc_df, time_limit_s=50)

            if pdc_energy is not None and mpc_energy is not None:
                energy_improvement = ((pdc_energy['total_energy_kj'] - mpc_energy['total_energy_kj']) / pdc_energy['total_energy_kj']) * 100

                print(f"\\n5 kph FV - {pdc_file} vs {mpc_file}")
                print(f"  PD Energy: {pdc_energy['total_energy_kj']:.2f} kJ | Avg Power: {pdc_energy['avg_power_w']:.2f} W")
                print(f"  MPC Energy: {mpc_energy['total_energy_kj']:.2f} kJ | Avg Power: {mpc_energy['avg_power_w']:.2f} W")
                print(f"  Energy Improvement: {energy_improvement:+.2f}%")

                all_energy_results_5kph.append({
                    'Speed': '5kph',
                    'PD Energy (kJ)': pdc_energy['total_energy_kj'],
                    'MPC Energy (kJ)': mpc_energy['total_energy_kj'],
                    'PD Avg Power (W)': pdc_energy['avg_power_w'],
                    'MPC Avg Power (W)': mpc_energy['avg_power_w'],
                    'Improvement (%)': energy_improvement,
                })

    if all_energy_results_5kph:
        energy_df_5kph = pd.DataFrame(all_energy_results_5kph)
        print("\\n" + "="*90)
        print("5 kph FV Energy Results")
        print("="*90)
        print(energy_df_5kph.to_string(index=False))
"""
    ]
}

# Create FV 10kph energy analysis cell
cell_energy_10kph_fv = {
    "cell_type": "code",
    "id": str(uuid.uuid4()),
    "metadata": {},
    "source": [
"""# ENERGY CONSUMPTION - 10 KPH FOLLOWING VEHICLE SCENARIO

# Get 10 kph files from both controllers
pdc_10kph_fv_files = [f for f in os.listdir(internal_PDC_FV_data_dir) if '10kph' in f]
mpc_10kph_fv_files = [f for f in os.listdir(internal_MPC_FV_data_dir_2) if '10kph' in f]

print(f"PD 10kph FV files: {pdc_10kph_fv_files}")
print(f"MPC 10kph FV files: {mpc_10kph_fv_files}")

if pdc_10kph_fv_files and mpc_10kph_fv_files:
    all_energy_results_10kph = []

    for pdc_file in pdc_10kph_fv_files:
        for mpc_file in mpc_10kph_fv_files:
            pdc_df = pd.read_csv(os.path.join(internal_PDC_FV_data_dir, pdc_file))
            mpc_df = pd.read_csv(os.path.join(internal_MPC_FV_data_dir_2, mpc_file))

            pdc_energy = calculate_energy_consumption(pdc_df, time_limit_s=50)
            mpc_energy = calculate_energy_consumption(mpc_df, time_limit_s=50)

            if pdc_energy is not None and mpc_energy is not None:
                energy_improvement = ((pdc_energy['total_energy_kj'] - mpc_energy['total_energy_kj']) / pdc_energy['total_energy_kj']) * 100

                print(f"\\n10 kph FV - {pdc_file} vs {mpc_file}")
                print(f"  PD Energy: {pdc_energy['total_energy_kj']:.2f} kJ | Avg Power: {pdc_energy['avg_power_w']:.2f} W")
                print(f"  MPC Energy: {mpc_energy['total_energy_kj']:.2f} kJ | Avg Power: {mpc_energy['avg_power_w']:.2f} W")
                print(f"  Energy Improvement: {energy_improvement:+.2f}%")

                all_energy_results_10kph.append({
                    'Speed': '10kph',
                    'PD Energy (kJ)': pdc_energy['total_energy_kj'],
                    'MPC Energy (kJ)': mpc_energy['total_energy_kj'],
                    'PD Avg Power (W)': pdc_energy['avg_power_w'],
                    'MPC Avg Power (W)': mpc_energy['avg_power_w'],
                    'Improvement (%)': energy_improvement,
                })

    if all_energy_results_10kph:
        energy_df_10kph = pd.DataFrame(all_energy_results_10kph)
        print("\\n" + "="*90)
        print("10 kph FV Energy Results")
        print("="*90)
        print(energy_df_10kph.to_string(index=False))
"""
    ]
}

# Add cells to FV notebook
nb_fv['cells'].extend([cell_energy_header, cell_energy_function, cell_energy_5kph_fv, cell_energy_10kph_fv])

# Save updated notebook
with open('PDC_VS_MPC_Comparison_FV.ipynb', 'w') as f:
    json.dump(nb_fv, f, indent=1)

print("[OK] Added energy consumption cells to FV notebook (4 cells)")
print("     - Markdown header")
print("     - Energy calculation function")
print("     - 5 kph FV energy analysis")
print("     - 10 kph FV energy analysis")
