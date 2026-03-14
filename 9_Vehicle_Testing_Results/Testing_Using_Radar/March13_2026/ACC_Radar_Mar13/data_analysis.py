import os
import re
import csv
import numpy as np

# ---------------- PATHS ----------------
script_dir = os.path.dirname(os.path.abspath(__file__))

# process all Test_* files automatically
log_files = [f for f in os.listdir(script_dir) if f.startswith("Test_")]

output_folder = os.path.join(script_dir, "extracted_csv")
os.makedirs(output_folder, exist_ok=True)

# ---------------- REGEX PATTERNS ----------------

time_pattern = re.compile(r'\[(\d+\.\d+)\]')

# format 1 (3.1 log)
speed_pattern = re.compile(r'VehicleSpeed:\s*([0-9\.\-eE]+)')
motor_pattern = re.compile(r'Received motor command:([0-9\.\-eE]+)')

# format 2 (2.2 log)
throttle_pattern = re.compile(r'The Throttle command is\s*:\s*([0-9\.\-eE]+)')
ego_pattern = re.compile(
    r'Absolute Ego states \(p,v,t\):\s*([0-9\.\-eE]+),([0-9\.\-eE]+),\s*([0-9\.\-eE]+)'
)

lead_pattern = re.compile(
    r'Absolute Lead pos:\s*(None|[0-9\.\-eE]+), Absolute lead vel :\s*(None|[0-9\.\-eE]+)'
)

sep_pattern = re.compile(
    r'Separation from lead vehicle\s*:\s*([0-9\.\-eE]+)'
)

warn_pattern = re.compile(r'\[WARN\]')

# ---------------- PROCESS FILES ----------------

for log_name in log_files:

    log_file = os.path.join(script_dir, log_name)
    output_csv = os.path.join(output_folder, log_name + ".csv")

    print("Processing:", log_name)

    data = []

    prev_time = None
    ego_pos_integrated = 0

    ego_pos = None
    ego_vel = None
    ego_t = None
    lead_pos = None
    lead_vel = None
    separation = None
    throttle = None
    warning = 0
    ros_time = None

    with open(log_file, 'r') as f:

        for line in f:

            # -------- ROS TIME --------
            tmatch = time_pattern.search(line)
            if tmatch:
                ros_time = float(tmatch.group(1))

            # -------- FORMAT 1 --------
            speed_match = speed_pattern.search(line)
            if speed_match:
                ego_vel = float(speed_match.group(1))

                if prev_time is not None:
                    dt = ros_time - prev_time
                    ego_pos_integrated += ego_vel * dt

                ego_pos = ego_pos_integrated
                prev_time = ros_time

            motor_match = motor_pattern.search(line)
            if motor_match:
                throttle = float(motor_match.group(1))

            # -------- FORMAT 2 --------
            throttle_match = throttle_pattern.search(line)
            if throttle_match:
                throttle = float(throttle_match.group(1))

            ego_match = ego_pattern.search(line)
            if ego_match:
                ego_pos = float(ego_match.group(1))
                ego_vel = float(ego_match.group(2))
                ego_t = float(ego_match.group(3))

            lead_match = lead_pattern.search(line)
            if lead_match:

                if lead_match.group(1) == "None":
                    lead_pos = np.nan
                else:
                    lead_pos = float(lead_match.group(1))

                if lead_match.group(2) == "None":
                    lead_vel = np.nan
                else:
                    lead_vel = float(lead_match.group(2))

            sep_match = sep_pattern.search(line)
            if sep_match:
                separation = float(sep_match.group(1))

            if warn_pattern.search(line):
                warning = 1

            # -------- SAVE ROW --------
            if ego_vel is not None and ros_time is not None:

                data.append([
                    ros_time,
                    ego_pos,
                    ego_vel,
                    lead_pos,
                    lead_vel,
                    separation,
                    throttle,
                    warning
                ])

                warning = 0

    # -------- REMOVE DUPLICATES --------
    unique_rows = []
    last_time = None

    for row in data:
        if row[0] != last_time:
            unique_rows.append(row)
            last_time = row[0]

    # -------- SAVE CSV --------
    header = [
        "ros_time",
        "ego_position",
        "ego_velocity",
        "lead_position",
        "lead_velocity",
        "separation",
        "throttle_command",
        "warning"
    ]

    with open(output_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(unique_rows)

    print("CSV saved:", output_csv)

print("\nAll logs processed successfully.")