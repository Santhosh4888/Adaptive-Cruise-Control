import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import signal
import os
import re

# Setup - use absolute paths
script_dir = os.path.dirname(os.path.abspath(__file__))
internal_PDC_CDP_data_dir = os.path.join(script_dir, "Processed_Data_March27_PD")
internal_MPC_CDP_data_dir = os.path.join(script_dir, "Processed_Data_March27_MPC")

sns.set_style("whitegrid")
plt.rcParams['figure.figsize'] = (14, 10)

def extract_file_info(filename):
    match = re.search(r'(\d+kph)_CS(\d+)', filename)
    if match:
        return match.group(1), int(match.group(2))
    return None, None

def calculate_metrics(df, controller_name="Controller"):
    sep_col = "Separation_clean"
    ego_vel_col = "Ego_Velocity_clean"
    obs_vel_col = "Obs_Velocity_clean"
    ego_acc_raw = "Ego_Acceleration_raw(m/s^2)"
    ego_acc_clean = "Ego_Acceleration(m/s^2)"
    time_col = "Time(s)"

    min_sep = df[sep_col].min()
    mean_sep = df[sep_col].mean()
    std_sep = df[sep_col].std()

    rms_acc = np.sqrt(np.mean(df[ego_acc_clean]**2))

    acc_smooth = df[ego_acc_clean].values
    dt = np.diff(df[time_col].values)
    dt = np.median(dt[dt > 0])
    jerk = np.diff(acc_smooth) / dt
    mean_jerk = np.mean(np.abs(jerk))
    max_jerk = np.max(np.abs(jerk))

    # Overshoot/Undershoot
    ego_vel = df[ego_vel_col].values
    obs_vel = df[obs_vel_col].values
    vel_error = ego_vel - obs_vel

    overshoot_idx = vel_error > 0
    if np.sum(overshoot_idx) > 0:
        overshoot_pct = (vel_error[overshoot_idx] / (obs_vel[overshoot_idx] + 1e-6)) * 100
        max_overshoot = np.max(overshoot_pct)
        mean_overshoot = np.mean(overshoot_pct)
    else:
        max_overshoot = 0
        mean_overshoot = 0

    undershoot_idx = vel_error < 0
    if np.sum(undershoot_idx) > 0:
        undershoot_pct = (np.abs(vel_error[undershoot_idx]) / (obs_vel[undershoot_idx] + 1e-6)) * 100
        max_undershoot = np.max(undershoot_pct)
        mean_undershoot = np.mean(undershoot_pct)
    else:
        max_undershoot = 0
        mean_undershoot = 0

    return {
        'Controller': controller_name,
        'Min Separation': min_sep,
        'Mean Separation': mean_sep,
        'Std Separation': std_sep,
        'RMS Acceleration': rms_acc,
        'Mean Jerk': mean_jerk,
        'Max Jerk': max_jerk,
        'Max Overshoot': max_overshoot,
        'Mean Overshoot': mean_overshoot,
        'Max Undershoot': max_undershoot,
        'Mean Undershoot': mean_undershoot,
    }

# Get all files
pdc_files = sorted([f for f in os.listdir(internal_PDC_CDP_data_dir) if f.endswith('.csv')])
mpc_files = sorted([f for f in os.listdir(internal_MPC_CDP_data_dir) if f.endswith('.csv')])

pdc_by_test = {}
mpc_by_test = {}
for f in pdc_files:
    speed, case = extract_file_info(f)
    if speed:
        pdc_by_test[(speed, case)] = f
for f in mpc_files:
    speed, case = extract_file_info(f)
    if speed:
        mpc_by_test[(speed, case)] = f

# Calculate all metrics
all_metrics = []
speeds = sorted(set([k[0] for k in pdc_by_test.keys()] + [k[0] for k in mpc_by_test.keys()]))

for speed in speeds:
    pdc_cases = sorted([k[1] for k in pdc_by_test.keys() if k[0] == speed])
    mpc_cases = sorted([k[1] for k in mpc_by_test.keys() if k[0] == speed])
    common_cases = sorted(set(pdc_cases) & set(mpc_cases))

    for case_num in common_cases:
        pdc_file = pdc_by_test[(speed, case_num)]
        mpc_file = mpc_by_test[(speed, case_num)]

        pdc_df = pd.read_csv(os.path.join(internal_PDC_CDP_data_dir, pdc_file))
        mpc_df = pd.read_csv(os.path.join(internal_MPC_CDP_data_dir, mpc_file))

        pdc_metrics = calculate_metrics(pdc_df, f"PD-{speed}-CS{case_num}")
        mpc_metrics = calculate_metrics(mpc_df, f"MPC-{speed}-CS{case_num}")

        pdc_metrics['Speed'] = speed
        mpc_metrics['Speed'] = speed
        pdc_metrics['Type'] = 'PD'
        mpc_metrics['Type'] = 'MPC'
        pdc_metrics['Case'] = f"CS{case_num}"
        mpc_metrics['Case'] = f"CS{case_num}"

        all_metrics.append(pdc_metrics)
        all_metrics.append(mpc_metrics)

metrics_df = pd.DataFrame(all_metrics)

# ============================================================================
# PLOT 1: Separation Metrics Comparison
# ============================================================================
fig, axes = plt.subplots(1, 2, figsize=(14, 5))

for idx, speed in enumerate(sorted(metrics_df['Speed'].unique())):
    speed_data = metrics_df[metrics_df['Speed'] == speed]
    pd_data = speed_data[speed_data['Type'] == 'PD'].sort_values('Case')
    mpc_data = speed_data[speed_data['Type'] == 'MPC'].sort_values('Case')

    cases = pd_data['Case'].values
    x = np.arange(len(cases))
    width = 0.35

    axes[idx].bar(x - width/2, pd_data['Min Separation'].values, width, label='PD', alpha=0.8, color='#1f77b4')
    axes[idx].bar(x + width/2, mpc_data['Min Separation'].values, width, label='MPC', alpha=0.8, color='#ff7f0e')

    axes[idx].set_xlabel('Test Case', fontsize=11, fontweight='bold')
    axes[idx].set_ylabel('Minimum Separation (m)', fontsize=11, fontweight='bold')
    axes[idx].set_title(f'{speed} Speed Profile', fontsize=12, fontweight='bold')
    axes[idx].set_xticks(x)
    axes[idx].set_xticklabels(cases)
    axes[idx].legend()
    axes[idx].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(script_dir, '1_Separation_Comparison.png'), dpi=300, bbox_inches='tight')
print("[1/5] Saved: 1_Separation_Comparison.png - Min separation across all test cases")

# ============================================================================
# PLOT 2: Acceleration Smoothness Metrics
# ============================================================================
fig, axes = plt.subplots(1, 2, figsize=(14, 5))

for idx, metric in enumerate(['RMS Acceleration', 'Mean Jerk']):
    speed_groups = sorted(metrics_df['Speed'].unique())
    pd_means = [metrics_df[(metrics_df['Speed'] == speed) & (metrics_df['Type'] == 'PD')][metric].mean() for speed in speed_groups]
    mpc_means = [metrics_df[(metrics_df['Speed'] == speed) & (metrics_df['Type'] == 'MPC')][metric].mean() for speed in speed_groups]

    x = np.arange(len(speed_groups))
    width = 0.35

    axes[idx].bar(x - width/2, pd_means, width, label='PD', alpha=0.8, color='#1f77b4')
    axes[idx].bar(x + width/2, mpc_means, width, label='MPC', alpha=0.8, color='#ff7f0e')

    axes[idx].set_xlabel('Speed Profile', fontsize=11, fontweight='bold')
    axes[idx].set_ylabel(metric + (' (m/s²)' if 'Accel' in metric else ' (m/s³)'), fontsize=11, fontweight='bold')
    axes[idx].set_title(metric, fontsize=12, fontweight='bold')
    axes[idx].set_xticks(x)
    axes[idx].set_xticklabels(speed_groups)
    axes[idx].legend()
    axes[idx].grid(True, alpha=0.3, axis='y')

plt.tight_layout()
plt.savefig(os.path.join(script_dir, '2_Smoothness_Metrics.png'), dpi=300, bbox_inches='tight')
print("[2/5] Saved: 2_Smoothness_Metrics.png - Acceleration and jerk comparison")

# ============================================================================
# PLOT 3: Overshoot/Undershoot Comparison
# ============================================================================
fig, axes = plt.subplots(1, 2, figsize=(14, 5))

for idx, speed in enumerate(sorted(metrics_df['Speed'].unique())):
    speed_data = metrics_df[metrics_df['Speed'] == speed]

    cases = speed_data[speed_data['Type'] == 'PD']['Case'].values
    x = np.arange(len(cases))
    width = 0.2

    pd_overshoot = speed_data[speed_data['Type'] == 'PD']['Max Overshoot'].values
    mpc_overshoot = speed_data[speed_data['Type'] == 'MPC']['Max Overshoot'].values
    pd_undershoot = speed_data[speed_data['Type'] == 'PD']['Max Undershoot'].values
    mpc_undershoot = speed_data[speed_data['Type'] == 'MPC']['Max Undershoot'].values

    axes[idx].bar(x - width*1.5, pd_overshoot, width, label='PD Overshoot', alpha=0.8, color='#d62728')
    axes[idx].bar(x - width*0.5, mpc_overshoot, width, label='MPC Overshoot', alpha=0.8, color='#ff7f0e')
    axes[idx].bar(x + width*0.5, pd_undershoot, width, label='PD Undershoot', alpha=0.8, color='#2ca02c')
    axes[idx].bar(x + width*1.5, mpc_undershoot, width, label='MPC Undershoot', alpha=0.8, color='#1f77b4')

    axes[idx].set_xlabel('Test Case', fontsize=11, fontweight='bold')
    axes[idx].set_ylabel('Max Percentage (%)', fontsize=11, fontweight='bold')
    axes[idx].set_title(f'{speed} - Overshoot/Undershoot', fontsize=12, fontweight='bold')
    axes[idx].set_xticks(x)
    axes[idx].set_xticklabels(cases)
    axes[idx].legend(fontsize=9)
    axes[idx].grid(True, alpha=0.3, axis='y')

plt.tight_layout()
plt.savefig(os.path.join(script_dir, '3_Overshoot_Undershoot.png'), dpi=300, bbox_inches='tight')
print("[3/5] Saved: 3_Overshoot_Undershoot.png - Velocity tracking errors")

# ============================================================================
# PLOT 4: Box Plots for Statistical Comparison
# ============================================================================
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
metrics_to_plot = ['Min Separation', 'RMS Acceleration', 'Mean Jerk', 'Max Overshoot']

for idx, metric in enumerate(metrics_to_plot):
    ax = axes[idx // 2, idx % 2]

    plot_data = []
    labels = []
    for speed in sorted(metrics_df['Speed'].unique()):
        for controller in ['PD', 'MPC']:
            data = metrics_df[(metrics_df['Speed'] == speed) & (metrics_df['Type'] == controller)][metric].values
            plot_data.append(data)
            labels.append(f"{speed}\n{controller}")

    bp = ax.boxplot(plot_data, labels=labels, patch_artist=True)

    # Color alternating boxes
    colors = ['#1f77b4', '#ff7f0e'] * len(sorted(metrics_df['Speed'].unique()))
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)

    ax.set_ylabel(metric, fontsize=11, fontweight='bold')
    ax.set_title(f'Distribution: {metric}', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')

plt.tight_layout()
plt.savefig(os.path.join(script_dir, '4_Statistical_Distribution.png'), dpi=300, bbox_inches='tight')
print("[4/5] Saved: 4_Statistical_Distribution.png - Box plots for all metrics")

# ============================================================================
# PLOT 5: Overall Performance Radar Chart
# ============================================================================
from math import pi

fig, axes = plt.subplots(1, 2, figsize=(14, 7))

speeds = sorted(metrics_df['Speed'].unique())
for speed_idx, speed in enumerate(speeds):
    ax = axes[speed_idx]

    speed_data = metrics_df[metrics_df['Speed'] == speed]

    # Metrics for radar chart (normalized 0-1)
    categories = ['Min Sep', 'RMS Accel', 'Mean Jerk', 'Max Overshoot', 'Max Undershoot']

    # Get averages for PD and MPC
    pd_avg = [
        speed_data[speed_data['Type'] == 'PD']['Min Separation'].mean() /
            speed_data['Min Separation'].max(),
        1 - (speed_data[speed_data['Type'] == 'PD']['RMS Acceleration'].mean() /
            speed_data['RMS Acceleration'].max()),
        1 - (speed_data[speed_data['Type'] == 'PD']['Mean Jerk'].mean() /
            speed_data['Mean Jerk'].max()),
        1 - (speed_data[speed_data['Type'] == 'PD']['Max Overshoot'].mean() /
            (speed_data['Max Overshoot'].max() + 0.1)),
        1 - (speed_data[speed_data['Type'] == 'PD']['Max Undershoot'].mean() /
            (speed_data['Max Undershoot'].max() + 0.1))
    ]

    mpc_avg = [
        speed_data[speed_data['Type'] == 'MPC']['Min Separation'].mean() /
            speed_data['Min Separation'].max(),
        1 - (speed_data[speed_data['Type'] == 'MPC']['RMS Acceleration'].mean() /
            speed_data['RMS Acceleration'].max()),
        1 - (speed_data[speed_data['Type'] == 'MPC']['Mean Jerk'].mean() /
            speed_data['Mean Jerk'].max()),
        1 - (speed_data[speed_data['Type'] == 'MPC']['Max Overshoot'].mean() /
            (speed_data['Max Overshoot'].max() + 0.1)),
        1 - (speed_data[speed_data['Type'] == 'MPC']['Max Undershoot'].mean() /
            (speed_data['Max Undershoot'].max() + 0.1))
    ]

    # Compute angle for each axis
    angles = [n / len(categories) * 2 * pi for n in range(len(categories))]
    pd_avg += pd_avg[:1]
    mpc_avg += mpc_avg[:1]
    angles += angles[:1]

    ax = plt.subplot(1, 2, speed_idx + 1, projection='polar')
    ax.plot(angles, pd_avg, 'o-', linewidth=2, label='PD', color='#1f77b4')
    ax.fill(angles, pd_avg, alpha=0.25, color='#1f77b4')
    ax.plot(angles, mpc_avg, 'o-', linewidth=2, label='MPC', color='#ff7f0e')
    ax.fill(angles, mpc_avg, alpha=0.25, color='#ff7f0e')

    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(categories, fontsize=10)
    ax.set_ylim(0, 1)
    ax.set_title(f'{speed} Speed Profile', fontsize=12, fontweight='bold', pad=20)
    ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))
    ax.grid(True)

plt.tight_layout()
plt.savefig(os.path.join(script_dir, '5_Performance_Radar.png'), dpi=300, bbox_inches='tight')
print("[5/5] Saved: 5_Performance_Radar.png - Overall performance comparison")

print("\n" + "="*70)
print("All plots generated successfully!")
print("="*70)
print("\nGenerated visualization files:")
print("  1. 1_Separation_Comparison.png - Min separation across test cases")
print("  2. 2_Smoothness_Metrics.png - Acceleration & jerk metrics")
print("  3. 3_Overshoot_Undershoot.png - Velocity tracking errors")
print("  4. 4_Statistical_Distribution.png - Box plots for all metrics")
print("  5. 5_Performance_Radar.png - Radar charts for overall performance")
