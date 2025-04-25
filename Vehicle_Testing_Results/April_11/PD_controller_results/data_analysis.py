#! user/bin/env  python3

import os
import re
import pandas as pd
import matplotlib.pyplot as plt

# Define base directory and file names
base_dir = "/home/asl-laptop2/Documents/Auto_drive_data"
log_file_name = "April_25_data_4"
csv_file_name = "loggeddata_April25_pd_1.csv"

# Construct dynamic paths
log_file_path = os.path.join(base_dir, log_file_name)
csv_path = os.path.join(base_dir, csv_file_name)

# Read log file
with open(log_file_path, "r") as file:
    lines = file.readlines()

data = []

i = 0
while i < len(lines) - 1:
    line = lines[i]
    if "The control signal is" in line:
        match_signal = re.search(r"\[(\d+\.\d+)\]: The control signal is : ([\d\.\-eE]+)", line)
        if match_signal:
            timestamp = float(match_signal.group(1))
            voltage = float(match_signal.group(2))

            next_line = lines[i + 1]
            match_data = re.search(r"data\s*:\s*([\d\.\-eE]+),([\d\.\-eE]+),\s*([\d\.\-eE]+)", next_line)
            if match_data:
                distance = float(match_data.group(1))
                velocity = float(match_data.group(2))
                time = float(match_data.group(3))

                data.append({
                    "Log Timestamp": timestamp,
                    "Distance (m)": distance,
                    "Velocity (m/s)": velocity,
                    "Time (s)": time,
                    "Voltage (V)": voltage
                })
            i += 2
        else:
            i += 1
    else:
        i += 1

# Create DataFrame
df = pd.DataFrame(data)

# Normalize time
df["Time (s)"] -= df["Time (s)"].iloc[0]

# Save to CSV
df.to_csv(csv_path, index=False)
print(f"CSV saved to: {csv_path}")

# Plot Distance vs Time and Velocity vs Time
plt.figure(figsize=(12, 6))

# Distance vs Time
plt.subplot(2, 1, 1)
plt.plot(df["Time (s)"], df["Distance (m)"], label="Distance", color="blue")
plt.ylabel("Distance (m)")
plt.title("Distance vs Time")
plt.grid(True)
plt.legend()

# Velocity vs Time
plt.subplot(2, 1, 2)
plt.plot(df["Time (s)"], df["Velocity (m/s)"], label="Velocity", color="green")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity vs Time")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
