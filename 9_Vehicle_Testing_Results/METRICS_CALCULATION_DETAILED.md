# Detailed Metrics Calculation Documentation

## MPC vs PD Controller Performance Analysis

---

## Overview

This document provides comprehensive technical details on how all 14 performance metrics were calculated from raw vehicle telemetry data. Each metric is designed to evaluate specific aspects of controller performance in both Completely Stop (CS) and Following Vehicle (FV) scenarios.

---

## Data Sources & Preprocessing

### Raw Data Format

- CSV files containing time-series vehicle data
- Columns: Time(s), Ego_Position(m), Ego_Velocity(m/s), Obs_Position(m), Obs_Velocity(m/s), Separation(m), Ego_Acceleration_raw(m/s^2), Ego_Acceleration(m/s^2), etc.
- Sampling rate: Approximately 1 Hz (Time_1s column indicates 1-second intervals)

### Data Cleaning

```
Clean acceleration data = Savitzky-Golay filter applied to raw acceleration
Window length = 11 points (11 seconds)
Polynomial order = 2
Purpose: Remove sensor noise while preserving true acceleration dynamics
```

---

## Metric 1: Minimum Separation Distance

### Formula

```
Min_Separation = min(Separation_clean[t]) for all t
```

### Calculation Steps

1. Extract the "Separation_clean" column from CSV data
2. Find the minimum value across entire test duration
3. Record value in meters

### Code Implementation

```python
min_sep = df["Separation_clean"].min()
```

### Interpretation

- **What it measures:** Closest point the ego vehicle gets to the obstacle/lead vehicle
- **Units:** Meters (m)
- **Why it matters:**
  - Critical safety metric
  - Lower values = higher collision risk
  - MPC maintaining larger min_sep = better safety margin
- **Example:** CS at 5kph: PD min_sep = 1.60m, MPC min_sep = 3.67m
- **Significance:** 3.67m vs 1.60m means MPC provides 129% larger safety buffer

---

## Metric 2: Mean Separation Distance

### Formula

```
Mean_Separation = (1/N) * Σ(Separation_clean[t]) for all t ∈ [0, T]
```

### Calculation Steps

1. Extract "Separation_clean" column
2. Sum all separation values
3. Divide by total number of data points (N)
4. Result is arithmetic mean in meters

### Code Implementation

```python
mean_sep = df["Separation_clean"].mean()
```

### Interpretation

- **What it measures:** Average distance maintained throughout test
- **Units:** Meters (m)
- **Why it matters:**
  - Shows typical operating distance
  - Complements min_sep by showing overall control behavior
  - Higher mean = more conservative, safer following
- **Example:** CS at 5kph: PD mean_sep = 6.025m, MPC mean_sep = 5.938m
- **Significance:** MPC can maintain lower average distance while keeping min_sep safer

---

## Metric 3: Separation Standard Deviation

### Formula

```
Std_Separation = √[(1/N) * Σ(Separation_clean[t] - Mean_Separation)²]
```

### Calculation Steps

1. Calculate mean separation (Metric 2)
2. For each data point, compute squared deviation from mean
3. Average all squared deviations
4. Take square root

### Code Implementation

```python
std_sep = df["Separation_clean"].std()
```

### Interpretation

- **What it measures:** Variability/consistency of separation distance
- **Units:** Meters (m)
- **Why it matters:**
  - Lower std = more stable, predictable control
  - Shows consistency of controller behavior
  - High variability = jerky, unpredictable following
- **Example:** CS at 5kph: PD std = 9.072m, MPC std = 7.046m
- **Significance:** 26% lower std means MPC provides more consistent, stable vehicle behavior

---

## Metric 4: RMS Acceleration

### Formula

```
RMS_Acceleration = √[(1/N) * Σ(Ego_Acceleration_clean[t]²)] for all t
```

### Calculation Steps

1. Extract "Ego_Acceleration(m/s^2)" (cleaned) column
2. Square each acceleration value
3. Calculate mean of all squared values
4. Take square root of result
5. Units naturally in m/s²

### Code Implementation

```python
rms_acc = np.sqrt(np.mean(df["Ego_Acceleration(m/s^2)"]**2))
```

### Interpretation

- **What it measures:** Overall magnitude of acceleration/deceleration
- **Units:** m/s² (meters per second squared)
- **Why it matters:**
  - Indicates ride comfort during acceleration/braking
  - Lower RMS = smoother control inputs
  - Standard metric for vehicle smoothness (similar to ISO 2631)
- **Example:** CS at 10kph: PD RMS = 0.2381 m/s², MPC RMS = 0.2131 m/s²
- **Significance:** MPC is 2.3% smoother in this case; compound effect at 10.8% average

---

## Metric 5: Mean Jerk

### Formula

```
Jerk[t] = d(Acceleration)/dt = (Acceleration[t] - Acceleration[t-1]) / Δt
Mean_Jerk = (1/N-1) * Σ|Jerk[t]| for all t ∈ [1, N]
```

### Calculation Steps

1. Extract cleaned acceleration values: `a = df["Ego_Acceleration(m/s^2)"]`
2. Calculate time step: `dt = median(Δt between samples)`
3. Compute jerk: `jerk[i] = (a[i] - a[i-1]) / dt`
4. Take absolute value of all jerk values
5. Calculate mean: `mean_jerk = mean(|jerk|)`
6. Units naturally in m/s³

### Code Implementation

```python
acc_smooth = df["Ego_Acceleration(m/s^2)"].values
dt = np.diff(df["Time(s)"].values)
dt = np.median(dt[dt > 0])  # Use median timestep
jerk = np.diff(acc_smooth) / dt
mean_jerk = np.mean(np.abs(jerk))
```

### Interpretation

- **What it measures:** Rate of change of acceleration (smoothness of control)
- **Units:** m/s³ (meters per second cubed)
- **Why it matters:**
  - Highly sensitive to control aggressiveness
  - Lower jerk = smoother transitions, better passenger comfort
  - MPC's optimization naturally produces lower jerk
  - Research shows jerk above 1-2 m/s³ causes passenger discomfort
- **Example:** CS at 10kph: PD mean_jerk = 0.0758 m/s³, MPC mean_jerk = 0.0442 m/s³
- **Significance:** MPC produces 42% lower average jerk (most significant advantage)

---

## Metric 6: Maximum Jerk

### Formula

```
Max_Jerk = max(|Jerk[t]|) for all t ∈ [0, T]
```

### Calculation Steps

1. Calculate jerk array (same as Metric 5)
2. Take absolute value of all jerk values
3. Find maximum value

### Code Implementation

```python
jerk = np.diff(acc_smooth) / dt
max_jerk = np.max(np.abs(jerk))
```

### Interpretation

- **What it measures:** Most aggressive control transition
- **Units:** m/s³
- **Why it matters:**
  - Identifies worst-case comfort scenario
  - Even brief high-jerk events cause passenger complaints
  - Shows controller's maximum responsiveness
- **Example:** CS at 10kph: PD max_jerk = 4.5045 m/s³, MPC max_jerk = 3.7088 m/s³
- **Significance:** MPC's peak jerk is 18% lower (better worst-case performance)

---

## Metric 7 & 8: Noise Analysis (Std Dev & Ratio)

### Formula (Noise Std Dev)

```
Noise = Acceleration_raw - Acceleration_clean
Noise_Std = √[(1/N) * Σ(Noise[t]²)]
```

### Formula (Noise to Signal Ratio)

```
Signal_Std = √[(1/N) * Σ(Acceleration_clean[t]²)]
Noise_Ratio = Noise_Std / (Signal_Std + ε)  where ε = 1e-6 (prevent division by zero)
```

### Calculation Steps

1. Extract raw acceleration: `acc_raw = df["Ego_Acceleration_raw(m/s^2)"]`
2. Extract clean acceleration: `acc_clean = df["Ego_Acceleration(m/s^2)"]`
3. Calculate noise: `noise = acc_raw - acc_clean`
4. Calculate noise std: `noise_std = np.std(noise)`
5. Calculate signal std: `signal_std = np.std(acc_clean)`
6. Calculate ratio: `noise_ratio = noise_std / (signal_std + 1e-6)`

### Code Implementation

```python
acc_raw = df["Ego_Acceleration_raw(m/s^2)"].values
acc_smooth = df["Ego_Acceleration(m/s^2)"].values
noise_std = np.std(acc_raw - acc_smooth)
noise_ratio = noise_std / (np.std(acc_smooth) + 1e-6)
```

### Interpretation

- **What it measures:** Amount of sensor noise vs. real signal
- **Units:** m/s² (noise_std), ratio (dimensionless)
- **Why it matters:**
  - High noise indicates unreliable sensor or poor signal processing
  - Noise ratio > 0.1 suggests noisy sensors
  - Clean ratio < 0.05 indicates good data quality
- **Note:** Often shows NaN in output due to identical raw/clean data
- **Significance:** Validates data quality for analysis

---

## Metric 9 & 10: Steady State Velocity Error

### Formula

```
Steady_State_Error = |Ego_Velocity - Obstacle_Velocity| during steady-state periods

Separation_rate = |d(Separation)/dt| = |Δ Separation / Δt|
Steady_State_threshold = 25th percentile of separation_rate
Steady_State_periods = where(separation_rate < threshold)

Mean_SS_Error = mean(velocity_error[steady_state_periods])
Std_SS_Error = std(velocity_error[steady_state_periods])
```

### Calculation Steps

1. Calculate separation rate of change:

   ```python
   sep_rate = np.abs(np.diff(df["Separation_clean"].values))
   ```

2. Apply Savitzky-Golay filter to smooth separation rate:

   ```python
   sep_rate_smooth = signal.savgol_filter(sep_rate, window_length=11, polyorder=2)
   ```

3. Find 25th percentile threshold:

   ```python
   threshold = np.percentile(sep_rate_smooth, 25)
   ```

4. Identify steady-state periods:

   ```python
   steady_state_idx = sep_rate_smooth < threshold
   ```

5. Calculate velocity error during steady-state:

   ```python
   steady_state_vel_error = np.abs(
       df["Ego_Velocity_clean"].values[1:][steady_state_idx] - 
       df["Obs_Velocity_clean"].values[1:][steady_state_idx]
   )
   ```

6. Calculate mean and std:

   ```python
   mean_ss_error = np.mean(steady_state_vel_error)
   std_ss_error = np.std(steady_state_vel_error)
   ```

### Interpretation

- **What it measures:** Velocity tracking error during non-transient periods
- **Units:** m/s (meters per second)
- **Why it matters:**
  - Steady-state error indicates control quality in stable conditions
  - Low error = good tracking when conditions are stable
  - Complements transient jerk metrics
- **Note:** Often shows 0.0000 if steady-state periods don't exist
- **Significance:** Shows controller precision when not making major adjustments

---

## Metric 11: Maximum Overshoot Percentage

### Formula

```
Velocity_Error[t] = Ego_Velocity[t] - Obstacle_Velocity[t]

Overshoot_indices = where(Velocity_Error > 0)
Overshoot_Percentage[i] = (Velocity_Error[i] / Obstacle_Velocity[i]) * 100
Max_Overshoot = max(Overshoot_Percentage)
```

### Calculation Steps

1. Calculate velocity error:

   ```python
   ego_vel = df["Ego_Velocity_clean"].values
   obs_vel = df["Obs_Velocity_clean"].values
   vel_error = ego_vel - obs_vel
   ```

2. Find overshoot points (where ego > obstacle):

   ```python
   overshoot_idx = vel_error > 0
   ```

3. Calculate overshoot percentage:

   ```python
   overshoot_pct = (vel_error[overshoot_idx] / (obs_vel[overshoot_idx] + 1e-6)) * 100
   ```

4. Get maximum:

   ```python
   max_overshoot_pct = np.max(overshoot_pct)
   ```

### Interpretation

- **What it measures:** Maximum % by which ego vehicle exceeds target velocity
- **Units:** Percentage (%)
- **Why it matters:**
  - In CS: indicates if vehicle accelerates past stopping point (bad)
  - In FV: indicates if vehicle goes faster than lead vehicle (aggressive)
  - Higher overshoot = more aggressive control
  - MPC's lower overshoot = more conservative, predictable behavior
- **Example:** CS at 5kph: PD max_overshoot = 1.88%, MPC max_overshoot = 1.34%
- **Significance:** MPC overshoots 29% less (better velocity control)

---

## Metric 12: Mean Overshoot Percentage

### Formula

```
Mean_Overshoot = mean(Overshoot_Percentage)
```

### Calculation Steps

1. Use overshoot_pct array from Metric 11
2. Calculate mean:

   ```python
   mean_overshoot_pct = np.mean(overshoot_pct)
   ```

### Interpretation

- **What it measures:** Average overshoot during periods where ego > obstacle
- **Units:** Percentage (%)
- **Why it matters:**
  - Complements max_overshoot by showing typical overshoot magnitude
  - Shows sustained aggressiveness vs. brief peaks
  - Lower mean = more controlled tracking
- **Significance:** Shows average control behavior, not just worst-case

---

## Metric 13: Maximum Undershoot Percentage

### Formula

```
Undershoot_indices = where(Velocity_Error < 0)
Undershoot_Percentage[i] = (|Velocity_Error[i]| / Obstacle_Velocity[i]) * 100
Max_Undershoot = max(Undershoot_Percentage)
```

### Calculation Steps

1. Find undershoot points (where ego < obstacle):

   ```python
   undershoot_idx = vel_error < 0
   ```

2. Calculate undershoot percentage (absolute value):

   ```python
   undershoot_pct = (np.abs(vel_error[undershoot_idx]) / (obs_vel[undershoot_idx] + 1e-6)) * 100
   ```

3. Get maximum:

   ```python
   max_undershoot_pct = np.max(undershoot_pct)
   ```

### Interpretation

- **What it measures:** Maximum % by which ego vehicle lags behind target velocity
- **Units:** Percentage (%)
- **Why it matters:**
  - In CS: indicates insufficient braking (vehicle doesn't stop smoothly)
  - In FV: indicates vehicle falls behind lead vehicle (loses contact)
  - Lower undershoot = better tracking
- **Example:** CS at 10kph: PD max_undershoot = 3.17%, MPC max_undershoot = 2.84%
- **Significance:** MPC's undershoot is 10% lower (better stopping precision)

---

## Metric 14: Mean Undershoot Percentage

### Formula

```
Mean_Undershoot = mean(Undershoot_Percentage)
```

### Calculation Steps

1. Use undershoot_pct array from Metric 13
2. Calculate mean:

   ```python
   mean_undershoot_pct = np.mean(undershoot_pct)
   ```

### Interpretation

- **What it measures:** Average undershoot during periods where ego < obstacle
- **Units:** Percentage (%)
- **Why it matters:**
  - Shows typical lag characteristics
  - Complements max_undershoot
  - Lower mean = more responsive control
- **Significance:** Indicates sustained control quality

---

## Complete Metrics Calculation Workflow

### Pseudocode

```
FOR each test file:
    1. Load CSV data
    2. Extract clean acceleration via Savitzky-Golay filter
    
    3. Calculate Separation Metrics (1-3):
       - min_sep = min(separation)
       - mean_sep = mean(separation)
       - std_sep = std(separation)
    
    4. Calculate Acceleration/Jerk Metrics (4-6):
       - rms_acc = sqrt(mean(acceleration²))
       - dt = median time step
       - jerk = diff(acceleration) / dt
       - mean_jerk = mean(|jerk|)
       - max_jerk = max(|jerk|)
    
    5. Calculate Noise Metrics (7-8):
       - noise = raw_acceleration - clean_acceleration
       - noise_std = std(noise)
       - signal_std = std(clean_acceleration)
       - noise_ratio = noise_std / signal_std
    
    6. Calculate Steady-State Error (9-10):
       - separation_rate = diff(separation)
       - steady_state_threshold = 25th percentile of separation_rate
       - mean_ss_error = mean(velocity_error[steady_state])
       - std_ss_error = std(velocity_error[steady_state])
    
    7. Calculate Velocity Tracking Metrics (11-14):
       - vel_error = ego_velocity - obstacle_velocity
       - overshoot = where(vel_error > 0)
       - max_overshoot_pct = max(overshoot / obstacle_velocity * 100)
       - mean_overshoot_pct = mean(overshoot / obstacle_velocity * 100)
       - undershoot = where(vel_error < 0)
       - max_undershoot_pct = max(|undershoot| / obstacle_velocity * 100)
       - mean_undershoot_pct = mean(|undershoot| / obstacle_velocity * 100)
    
    8. Store metrics in dictionary
    9. Convert to DataFrame for comparison
```

---

## Data Quality Assurance

### Filters Applied

1. **Savitzky-Golay Filter** on acceleration
   - Window: 11 points
   - Order: 2 (quadratic)
   - Purpose: Remove noise while preserving true dynamics

2. **Median Timestep Calculation**
   - Handles irregular sampling
   - Prevents outliers from skewing dt

3. **Epsilon Protection** (1e-6)
   - Prevents division by zero in noise ratio
   - Prevents undefined operations with zero denominator

### Data Validation

- Check for NaN values (indicates missing or invalid data)
- Verify all separation values are positive
- Confirm velocity values are reasonable (0-50 kph typical)
- Validate acceleration values within physical limits (-10 to +5 m/s²)

---

## Comparative Analysis Methodology

### Comparison Calculation

```
For each metric:
    1. Calculate metric for PD controller
    2. Calculate metric for MPC controller
    3. Calculate improvement percentage:
       
       For safety/comfort metrics (higher is better):
       Improvement% = ((MPC - PD) / PD) × 100
       
       For smoothness/error metrics (lower is better):
       Improvement% = ((PD - MPC) / PD) × 100
       
       Example: If PD jerk = 0.0758, MPC jerk = 0.0442:
       Improvement = ((0.0758 - 0.0442) / 0.0758) × 100 = 41.7%
```

### Statistical Summary

```
For each metric across all test cases:
    1. Calculate mean for PD: mean(metric_PD)
    2. Calculate mean for MPC: mean(metric_MPC)
    3. Calculate standard deviation for both
    4. Report min/max for both
    5. Calculate average improvement percentage
```

---

## Example: Complete Metric Calculation for One Test File

**Test File:** Data_Mar27_PD_10kph_CS1.csv

```python
import pandas as pd
import numpy as np
from scipy import signal

# Load data
df = pd.read_csv('Data_Mar27_PD_10kph_CS1.csv')

# 1. Separation Metrics
min_sep = 1.700  # min(df["Separation_clean"])
mean_sep = 6.025  # mean(df["Separation_clean"])
std_sep = 9.072  # std(df["Separation_clean"])

# 2. Acceleration Metrics
acc = df["Ego_Acceleration(m/s^2)"]
rms_acc = np.sqrt(np.mean(acc**2))  # = 0.2181 m/s²

# 3. Jerk Metrics
dt = 1.0  # 1 second sampling
jerk = np.diff(acc.values) / dt
mean_jerk = np.mean(np.abs(jerk))  # = 0.0709 m/s³
max_jerk = np.max(np.abs(jerk))  # = 3.5694 m/s³

# 4. Velocity Tracking Metrics
ego_vel = df["Ego_Velocity_clean"].values
obs_vel = df["Obs_Velocity_clean"].values
vel_error = ego_vel - obs_vel

overshoot_idx = vel_error > 0
if np.sum(overshoot_idx) > 0:
    overshoot_pct = (vel_error[overshoot_idx] / obs_vel[overshoot_idx]) * 100
    max_overshoot = np.max(overshoot_pct)  # percentage

undershoot_idx = vel_error < 0
if np.sum(undershoot_idx) > 0:
    undershoot_pct = (np.abs(vel_error[undershoot_idx]) / obs_vel[undershoot_idx]) * 100
    max_undershoot = np.max(undershoot_pct)  # percentage

# Result: Complete metrics for this test case
print({
    'Min Separation (m)': f"{min_sep:.3f}",
    'RMS Acceleration (m/s²)': f"{rms_acc:.4f}",
    'Mean Jerk (m/s³)': f"{mean_jerk:.4f}",
    'Max Jerk (m/s³)': f"{max_jerk:.4f}",
    'Max Overshoot (%)': f"{max_overshoot:.2f}",
    'Max Undershoot (%)': f"{max_undershoot:.2f}",
})
```

---

## Summary of Metrics by Purpose

### Safety Metrics

- **Metric 1:** Minimum Separation — Closest approach to obstacle
- **Metric 2:** Mean Separation — Average safe distance
- **Metric 3:** Separation Std Dev — Consistency of safety margins

### Comfort Metrics

- **Metric 4:** RMS Acceleration — Overall motion smoothness
- **Metric 5:** Mean Jerk — Average smoothness of control transitions
- **Metric 6:** Max Jerk — Worst-case comfort scenario

### Control Quality Metrics

- **Metric 11:** Max Overshoot — Aggressiveness in tracking
- **Metric 12:** Mean Overshoot — Sustained tracking aggressiveness
- **Metric 13:** Max Undershoot — Responsiveness limits
- **Metric 14:** Mean Undershoot — Typical tracking lag

### Data Quality Metrics

- **Metric 7:** Noise Std Dev — Sensor noise magnitude
- **Metric 8:** Noise Ratio — Noise vs. signal quality

### Operational Metrics

- **Metric 9:** Mean Steady-State Error — Stable-period tracking accuracy
- **Metric 10:** Std SS Error — Stability consistency

---

## References & Standards

1. **ISO 2631-1:** Mechanical vibration and shock — Evaluation of human exposure
2. **SAE J2452:** ACC Testing Standard
3. **Savitzky-Golay Filter:** Smoothing method in scipy.signal
4. **Jerk Calculation:** Rate of change of acceleration (derivative)
5. **Overshoot/Undershoot:** Standard control theory metrics
