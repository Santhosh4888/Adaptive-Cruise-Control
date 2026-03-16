import os
import re
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

internal_data_analysis_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.abspath(os.path.join(internal_data_analysis_dir, '..'))
internal_data_collection_dir = os.path.abspath(os.path.join(internal_data_analysis_dir, 'May_07'))
print(f'The directory of data analysis : {internal_data_analysis_dir}')
print(f'The root directory of this project : {root_dir}')
print(f'The directory of collected data : {internal_data_collection_dir}')

try:
    MPC_Data_dir = os.path.abspath(os.path.join(internal_data_collection_dir, 'MPC_controller_data'))
    if not os.path.exists(MPC_Data_dir):
        print(f"Folder not found: {MPC_Data_dir}")
        MPC_Data_dir = None
except Exception as e:
    print(f"Error with MPC_Data_dir: {e}")
    MPC_Data_dir = None
try:
    PD_data_dir = os.path.abspath(os.path.join(internal_data_collection_dir, 'PD_controller_data'))
    if not os.path.exists(PD_data_dir):
        print(f"Folder not found: {PD_data_dir}")
        PD_data_dir = None
except Exception as e:
    print(f"Error with PD_data_dir: {e}")
    PD_data_dir = None


if MPC_Data_dir is not None and os.path.exists(MPC_Data_dir):
    for file in os.listdir(MPC_Data_dir):
        print(os.path.join(MPC_Data_dir, file))
else:
    print("MPC_Data_dir folder is not there, skipping.")

if PD_data_dir is not None and os.path.exists(PD_data_dir):
    for file in os.listdir(PD_data_dir):
        print(os.path.join(PD_data_dir, file))
else:
    print("PD_data_dir folder is not there, skipping.")


# Load data from the CSV files in the specified directories
if MPC_Data_dir is not None and os.path.exists(MPC_Data_dir):
    mpc_data_files = [os.path.join(MPC_Data_dir, file) for file in os.listdir(MPC_Data_dir) if file.endswith('.csv')]
else:
    print("MPC_Data_dir folder is not there, Cannot load the data.")

if PD_data_dir is not None and os.path.exists(PD_data_dir):
    pd_data_files = [os.path.join(PD_data_dir, file) for file in os.listdir(PD_data_dir) if file.endswith('.csv')]
else:
    print("PD_data_dir folder is not there, Cannot load the data.")

# Load the data into dataframes
if 'mpc_data_files' in locals() and mpc_data_files:
    mpc_dataframes = [pd.read_csv(file) for file in mpc_data_files]
    print(f"Loaded {len(mpc_dataframes)} MPC data files.")
else:
    mpc_dataframes = []
    print("No MPC data files found, skipping loading MPC data.")

if 'pd_data_files' in locals() and pd_data_files:
    pd_dataframes = [pd.read_csv(file) for file in pd_data_files]
    print(f"Loaded {len(pd_dataframes)} PD data files.")
else:
    pd_dataframes = []
    print("No PD data files found, skipping loading PD data.")

# Select the DataFrame
df = pd_dataframes[2]

# Check if the required columns exist
if "Time(s)" in df.columns and "Ego_Position(m)" in df.columns and "Ego_Velocity(m/s)" in df.columns:
    # Initialize the figure and axes
    fig, ax = plt.subplots(1, 2, figsize=(20, 8))

    # Set up the first subplot for Distance vs Time
    ax[0].set_title("Distance vs Time")
    ax[0].set_xlabel("Time (s)")
    ax[0].set_ylabel("Ego Position (m)")
    ax[0].grid(True)
    line1, = ax[0].plot([], [], label="Ego Position", color="blue")
    ax[0].legend()

    # Set up the second subplot for Velocity vs Time
    ax[1].set_title("Velocity vs Time")
    ax[1].set_xlabel("Time (s)")
    ax[1].set_ylabel("Ego Velocity (m/s)")
    ax[1].grid(True)
    line2, = ax[1].plot([], [], label="Ego Velocity", color="green")
    ax[1].legend()

    # Initialize the data for animation
    def init():
        line1.set_data([], [])
        line2.set_data([], [])
        return line1, line2

    # Update function for animation
    def update(frame):
        # Update the data for the current frame
        time = df["Time(s)"][:frame]
        position = df["Ego_Position(m)"][:frame]
        velocity = df["Ego_Velocity(m/s)"][:frame]

        # Update the lines
        line1.set_data(time, position)
        line2.set_data(time, velocity)

        # Adjust the axis limits dynamically
        ax[0].set_xlim(0, df["Time(s)"].max())
        ax[0].set_ylim(0, df["Ego_Position(m)"].max() + 10)
        ax[1].set_xlim(0, df["Time(s)"].max())
        ax[1].set_ylim(0, df["Ego_Velocity(m/s)"].max() + 10)

        return line1, line2

    # Create the animation
    ani = FuncAnimation(fig, update, frames=len(df), init_func=init, blit=True, interval=100, repeat = False)

    #show the animation
    plt.tight_layout
    plt.show()
else:
    print("The required columns are not present in the DataFrame.")