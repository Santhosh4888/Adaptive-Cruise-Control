import numpy as np

# Direct variables

GEAR_RATIO = 10                                                            # Gear ratio of the vehicle
tyre_radius = 0.265                                                        # Tyre radius of the vehicle in meters
periodic_step = 0.1                                                        # Periodic step in seconds
ego_max_v = 20.0 * 5 / 18                                                  # Maximum velocity of ego vehicle in m/s
max_tcmd = 100.0                                                           # Maximum throttle command

# Indirect variables

max_rpm = ego_max_v * 60 * GEAR_RATIO / (2 * np.pi * tyre_radius)          # Maximum rpm of ego vehicle in RPM