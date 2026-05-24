import pandas as pd
import numpy as np
from pathlib import Path

# Load normalized data
data_folder = Path(".")
csv_files = sorted([f for f in data_folder.glob("*_normalized.csv")])

def calculate_energy_consumption_50s(df, mass_kg=680, mu_rolling=0.015, g=9.81, time_limit_s=50):
    df_filtered = df[df['time_seconds'] <= time_limit_s].copy()
    if len(df_filtered) < 2:
        return None
    
    time_orig = df_filtered['time_seconds'].values
    time_resampled = np.arange(0, time_limit_s + 0.1, 1.0)
    time_resampled = time_resampled[time_resampled <= time_limit_s]
    
    accel_orig = df_filtered['acceleration_clean'].values
    accel_resampled = np.interp(time_resampled, time_orig, accel_orig)
    
    velocity_orig = df_filtered['velocity_ms'].values
    velocity_resampled = np.interp(time_resampled, time_orig, velocity_orig)
    
    time = time_resampled
    accel = accel_resampled
    velocity = velocity_resampled
    
    dt = np.diff(time)
    F_friction = mu_rolling * mass_kg * g
    F_accel = mass_kg * accel
    
    power_accel = F_accel * velocity
    power_friction = F_friction * velocity
    power_total_unclamped = power_accel + power_friction
    power_total = np.maximum(power_total_unclamped, 0)
    
    power_accel_intervals = (power_accel[:-1] + power_accel[1:]) / 2
    power_friction_intervals = (power_friction[:-1] + power_friction[1:]) / 2
    power_total_intervals = (power_total[:-1] + power_total[1:]) / 2
    
    is_engine_on = power_total_intervals > 0
    
    total_energy_j = np.sum(power_total_intervals * dt)
    energy_accel_j = np.sum(power_accel_intervals[is_engine_on] * dt[is_engine_on])
    energy_friction_j = np.sum(power_friction_intervals[is_engine_on] * dt[is_engine_on])
    energy_braking_j = -np.sum(np.minimum(power_accel_intervals[~is_engine_on], 0) * dt[~is_engine_on])
    
    avg_power = total_energy_j / (time[-1] - time[0]) if len(time) > 1 else 0
    peak_power = np.max(power_total)
    energy_sum_j = energy_accel_j + energy_friction_j
    
    return {
        'total_energy_kj': total_energy_j / 1000,
        'energy_accel_kj': energy_accel_j / 1000,
        'energy_friction_kj': energy_friction_j / 1000,
        'energy_braking_kj': energy_braking_j / 1000,
        'avg_power_w': avg_power,
        'peak_power_w': peak_power,
        'energy_sum_kj': energy_sum_j / 1000,
        'duration_s': time[-1] - time[0],
        'samples': len(time),
    }

energy_50s_results = []

for csv_file in csv_files:
    df = pd.read_csv(csv_file)
    filename = csv_file.stem.replace('_normalized', '')
    
    energy_50s = calculate_energy_consumption_50s(df, time_limit_s=50)
    
    if energy_50s is None:
        print(f"[SKIPPED] {filename}: Insufficient data in first 50 seconds")
        continue
    
    energy_50s_results.append({
        'File': filename,
        'Total (kJ)': round(energy_50s['total_energy_kj'], 4),
        'Accel (kJ)': round(energy_50s['energy_accel_kj'], 4),
        'Friction (kJ)': round(energy_50s['energy_friction_kj'], 4),
        'Braking (kJ)': round(energy_50s['energy_braking_kj'], 4),
        'Avg Power (W)': round(energy_50s['avg_power_w'], 2),
        'Peak Power (W)': round(energy_50s['peak_power_w'], 2),
        'Duration (s)': round(energy_50s['duration_s'], 1),
        'Samples': energy_50s['samples'],
    })

energy_50s_df = pd.DataFrame(energy_50s_results)
print("="*100)
print("ENERGY CONSUMPTION - 50 SECOND WINDOW (FOR PDC vs MPC COMPARISON)")
print("="*100)
print(energy_50s_df.to_string(index=False))
print()

energy_50s_df.to_csv('Energy_Consumption_50s_Window.csv', index=False)
print("✓ Saved to: Energy_Consumption_50s_Window.csv")
