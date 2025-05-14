import os
import re
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation



internal_data_analysis_dir = os.path.dirname(os.path.abspath('Dynamic_plotting.ipynb'))
root_dir = os.path.abspath(os.path.join(internal_data_analysis_dir, '..'))
internal_data_collection_dir = os.path.abspath(os.path.join(internal_data_analysis_dir, 'May_07'))
print(f'The directory of data analysis : {internal_data_analysis_dir}')
print(f'The root directory of this project : {root_dir}')
print(f'The directory of collected data : {internal_data_collection_dir}')

PD_data_dir = os.path.abspath(os.path.join(internal_data_collection_dir, 'PD_controller_data'))

for file in os.listdir(PD_data_dir):
    print(os.path.join(PD_data_dir, file)) 


pd_data_files = [os.path.join(PD_data_dir, file) for file in os.listdir(PD_data_dir) if file.endswith('.csv')]


pd_dataframes = [pd.read_csv(file) for file in pd_data_files]


print(f"Loaded {len(pd_dataframes)} PD data files.")


# Display the first few rows of each PD dataframe
print("\nPD Dataframes:")
for i, df in enumerate(pd_dataframes):
    print(f"\nPD Dataframe {i+1}:")
    print(df.head())


# Select the DataFrame
df = pd_dataframes[4]

# Check if the required columns exist
if "Time(s)" in df.columns and "Ego_Position(m)" in df.columns and "Ego_Velocity(m/s)" in df.columns:
    # Initialize the figure and axes
    fig, ax = plt.subplots(2, 1, figsize=(10, 8))

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
    ani = FuncAnimation(fig, update, frames=len(df), init_func=init, blit=True, interval=50)

    # Embed the animation in the notebook
    #HTML(ani.to_jshtml())  # Use this to display the animation in the notebook
else:
    print("The required columns are not present in the DataFrame.")