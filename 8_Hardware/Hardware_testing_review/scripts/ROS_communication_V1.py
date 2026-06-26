#!/usr/bin/env python3
# This version is yet to be tested. In this we are also handling radar dropout 
# By tracking the last seen object for 2 sec if radar loses the object.
import numpy as np
import rospy
from std_msgs.msg import Float32, Bool
import H_Common_Params as CP
import Longitudinal_Controller as LC
import csv
import os


class Communication:
    
    def __init__(self):
        
        # Ego states
        self.ego_vel = 0.0
        self.ego_pos = 0.0
        # Radar states
        self.lead_distance = None
        self.lead_relative_velocity = 0.0
        self.lead_valid = False
        #self.prev_lead_valid = False
        #self.obs_vel = None                                                                       # Velocity of obstacle vehicle in m/s, given for now, once RADAR sensor is ready, this will be obtained directly from it 
        #self.obs_pos = None 
        # Last known obstacle (for dropout handling)
        self.last_obs_pos = None
        self.last_obs_vel = None
        self.last_seen_time = None
        self.obs_confidence = 0
        # Time
        self.start_time = None
        self.cur_time = None
        # Controller
        self.VLC = LC.VLC()
        # ROS publishers
        self.longitudinal_control_pub = None
        self.brake_control_pub = None
        # Data logging
        self.store_position = []
        self.store_velocity = []
        self.store_obs_pos = []
        self.store_obs_vel = []
        self.store_lead_distance = []
        self.store_lead_rel_vel = []
        self.store_time = []
        self.store_throttle_cmd = []
        
        self.save_dir = os.path.join(os.path.dirname(__file__), '..', 'data')
        self.file_path = os.path.join(self.save_dir, 'Data_MPC_Stable_March19.csv')
    

    # ---------------------- START ----------------------
    def start_vehicle(self):
        
        rospy.init_node('control_node', anonymous=True)
        rospy.loginfo("Node Started")
        rospy.Timer(
            rospy.Duration(CP.H_total_experiment_time),
            self.vehicle_shutdown_callback,
            oneshot=True
        )
        self.start_time = rospy.Time.now()
        self.cur_time = rospy.Time.now()
        self.last_seen_time = rospy.Time.now()
        self.create_publishers()
        self.start_subscribers()
        rospy.spin()


    # ---------------------- SUBSCRIBERS ----------------------
    def start_subscribers(self):
        
        rospy.Subscriber('/velocity_feedback', Float32, self.velocity_callback, queue_size=10)
        rospy.Subscriber('/lead_distance', Float32, self.lead_distance_callback, queue_size=10)
        rospy.Subscriber('/lead_relative_velocity', Float32, self.lead_relative_velocity_callback, queue_size=10)
        rospy.Subscriber('/lead_valid', Bool, self.lead_valid_callback, queue_size=10)


    # ---------------------- PUBLISHERS ----------------------
    def create_publishers(self):
        
        self.longitudinal_control_pub = rospy.Publisher('/motor_command', Float32, queue_size=10)
        self.brake_control_pub = rospy.Publisher('/brake_command', Bool, queue_size=10)


    # ---------------------- RADAR CALLBACKS ----------------------
    def lead_distance_callback(self, msg):
        self.lead_distance = msg.data  # In Meters

    def lead_relative_velocity_callback(self, msg):
        self.lead_relative_velocity = msg.data # In m/s

    def lead_valid_callback(self, msg):
        self.lead_valid = msg.data # True if lead is detected, False otherwise


    # ---------------------- MAIN CONTROL CALLBACK ----------------------
    def velocity_callback(self, msg):
        
        # --- TIME UPDATE ---
        new_time = rospy.Time.now()
        dt = (new_time - self.cur_time).to_sec()
        dt = max(dt, 1e-3)

        # --- UPDATE EGO STATE (FIXED ORDER) ---
        self.ego_vel = msg.data * 5/18  # kmph → m/s
        self.ego_vel = np.clip(self.ego_vel, 0, 10)  # sanity clamp

        self.ego_pos += self.ego_vel * dt
        self.cur_time = new_time


        # --- RADAR PROCESSING ---
        obs_pos, obs_vel = None, None
        if self.lead_valid and self.lead_distance is not None:

            # Clamp noisy radar values
            self.lead_distance = np.clip(self.lead_distance, 0, 40)
            self.lead_relative_velocity = np.clip(self.lead_relative_velocity, -10, 10)

            # Convert to absolute states
            obs_pos = self.ego_pos + self.lead_distance
            obs_vel = self.ego_vel + self.lead_relative_velocity

            # Store last valid
            self.last_obs_pos = obs_pos
            self.last_obs_vel = obs_vel
            self.last_seen_time = rospy.Time.now()
            self.obs_confidence += 1

        else:
            time_since_seen = (rospy.Time.now() - self.last_seen_time).to_sec()
            # Handle dropout → predict motion or hold last value
            if self.last_obs_pos is not None and time_since_seen < 2:
                obs_pos = self.last_obs_pos 
                obs_vel = self.last_obs_vel
                self.last_obs_pos = obs_pos # FIX: Predict forward instead of holding stale position

            else:
                # Switch to cruise
                obs_pos = None
                obs_vel = None
                self.obs_confidence = 0

        # --- SAFETY SUPERVISOR ---
        if obs_pos is not None:
            separation = obs_pos - self.ego_pos
            safe_dist = CP.Dd + CP.Td * self.ego_vel
            if separation < safe_dist:
                rospy.logwarn("Too close! Overriding throttle to zero")
                self.VLC.throttle_pot = 0.0
       
        # --- CONTROL ACTION ---
        if obs_pos is not None:
            self.VLC.get_control_action(
                [self.ego_pos, self.ego_vel],
                [obs_pos, obs_vel]  
            )
        else:
            self.VLC.get_control_action(
                [self.ego_pos, self.ego_vel],
                None  
            )

        # --- PUBLISH ---
        self.longitudinal_control_pub.publish(self.VLC.throttle_pot)


        # --- LOGGING ---
        self.store_position.append(self.ego_pos)
        self.store_velocity.append(self.ego_vel)
        self.store_obs_pos.append(obs_pos)
        self.store_obs_vel.append(obs_vel)
        self.store_lead_distance.append(self.lead_distance)
        self.store_lead_rel_vel.append(self.lead_relative_velocity)
        self.store_time.append((self.cur_time - self.start_time).to_sec())
        self.store_throttle_cmd.append(self.VLC.throttle_pot)


        # --- DEBUG PRINTS ---
        rospy.loginfo(f"Throttle: {self.VLC.throttle_pot} V")
        rospy.loginfo(f"Ego (pos, vel): {self.ego_pos:.2f}, {self.ego_vel:.2f}")
        
        if obs_pos is not None:
            rospy.loginfo(f"Lead (pos, vel): {obs_pos:.2f}, {obs_vel:.2f}")
            rospy.loginfo(f"Separation: {obs_pos - self.ego_pos:.2f}")
        else:
            rospy.loginfo("Lead: NOT AVAILABLE, I am in CRUISE MODE")


    # ---------------------- SHUTDOWN ----------------------
    def vehicle_shutdown_callback(self, event):
        
        rospy.loginfo("Saving data and shutting down...")

        with open(self.file_path, 'w', newline='') as file:
            writer = csv.writer(file)

            writer.writerow([
                "Ego_Position(m)",
                "Ego_Velocity(m/s)",
                "Obs_Position(m)",
                "Obs_Velocity(m/s)",
                "Lead_Distance(m)",
                "Lead_Rel_Vel(m/s)",
                "Time(s)",
                "Throttle_command"
            ])

            writer.writerows(zip(
                self.store_position,
                self.store_velocity,
                self.store_obs_pos,
                self.store_obs_vel,
                self.store_lead_distance,
                self.store_lead_rel_vel,
                self.store_time,
                self.store_throttle_cmd
            ))

        rospy.signal_shutdown("Test complete")


# ---------------------- MAIN ----------------------
if __name__ == '__main__':
    
    VC = Communication()
    VC.start_vehicle()