#!/usr/bin/env python3

import re
import pandas as pd 
import matplotlib.pyplot as plt

log_file_path = 'log.txt' #Actual file path will be added later

print("Opening  log file...")
with open(log_file_path, 'r') as file:
    contents = file.read()

print("File read successfully, extracting data...")

matches = re.findall(r"\[INFO] \[(\d+\.\d+)\]: The control signal is : ([\d.\-eE]+)", contents)
timestamps = [float(m[0]) for m in matches]
voltages = [float(m[1]) for m in matches]

print(f"Extracted {len(timestamps)} timestamps and {len(voltages)} voltage signals.")

print("Searching for the distance array logged at the end")
array_matches = re.findall(r"\[s*((?:[\d\.\-eE]+\s*,\s*)*[\d\.\-eE]+)\s*\]", contents)

all_arrays = []

for match in array_matches:
    try:
        values = [float(x.strip()) for x in match.split(',')]
        all_arrays.append(values)
    except:
        continue

distances = max(all_arrays, key=len) if all_arrays else []
print(f"Extracted distance array with {len(distances)} elements.")

min_length = min(len(timestamps), len(voltages), len(distances))
timestamps = timestamps[:min_length]        
voltages = voltages[:min_length]
distances = distances[:min_length]
print(f"Truncated all arrays to the minimum length of {min_length}.")

start_time = timestamps[0]
time_normalised = [t - start_time for t in timestamps]

df = pd.DataFrame({
    'Time (s)': time_normalised,
    'Voltage (V)': voltages,
    'Distance (m)': distances
})

csv_file_path = 'output.csv' #Actual file path will be added later
df.to_csv(csv_file_path, index=False)
print(f"Data saved to {csv_file_path}.")

plt.figure(figsize=(12, 6))
plt.plot(df['Time (s)'], df['Distance (m)'], label='Distance (m)', color='orange')
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')  
plt.title('Distance covered  vs Time')
plt.grid(True)
plt.legend()
plt.tight_layout()      
plt.show()
