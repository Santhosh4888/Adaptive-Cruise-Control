#!/usr/bin/env python3
#This file is under development
import numpy as np
import rospy
from std_msgs.msg import Float32, Bool
import H_Common_Params as CP
import Longitudinal_Controller as LC
import csv
import os

class Communication:
    
    def __init__(self):
        
        self.ego_vel = 0.0                              # Velocity of ego vehicle in m/s
        self.ego_pos = 0.0                              # Position of ego vehicle in meters, estimated for now, once LIDAR sensor is ready, this will be obtained directly from it
        self.lead_distance = None                       # Published by the Radar
        self.lead_relative_velocity = 0.0               # Published by the Radar
        self.lead_valid = False                         # Published by the Radar
        self.prev_lead_valid = False                    # Published by the Radar
        self.obs_vel = None                             # Velocity of obstacle vehicle in m/s, calculated from RADAR sensor data.
        self.obs_pos = None 
        self.start_time = None
        self.cur_time = None
        self.VLC = LC.VLC()                             # Getting the vehicle Longitudinal controller
        self.longitudinal_control_pub = None
        self.store_position = [self.ego_pos]
        self.store_velocity = [self.ego_vel]
        self.store_obs_pos = []
        self.store_obs_vel = []
        self.store_lead_distance = []
        self.store_lead_rel_vel = []
        self.store_time = []
        self.save_dir = os.path.join(os.path.dirname(__file__), '..', 'data')
        self.file_path = os.path.join(self.save_dir, 'Data_10kph_CS.csv')
        
    def start_vehicle(self):
        
        rospy.init_node('control_node', anonymous = True)
        print("Node Started")
        rospy.Timer(rospy.Duration(CP.H_total_experiment_time), self.vehicle_shutdown_callback, oneshot = True) # Sets the total experiment time
        self.start_time = rospy.Time.now()
        self.cur_time = rospy.Time.now()
        self.store_time.append((self.cur_time - self.start_time).to_sec())  #need to check
        self.create_publishers()
        self.start_subscribers()
        rospy.spin()
            
    def start_subscribers(self):     
        
        rospy.Subscriber('/velocity_feedback', Float32, self.velocity_callback, queue_size = 10)

        # Subscribers for radar data
        rospy.Subscriber('/lead_distance', Float32, self.lead_distance_callback, queue_size= 10) 
        rospy.Subscriber('/lead_relative_velocity', Float32, self.lead_relative_velocity_callback, queue_size= 10)
        rospy.Subscriber('/lead_valid', Bool, self.lead_valid_callback, queue_size=10)
    
    def create_publishers(self):
        
        self.longitudinal_control_pub = rospy.Publisher('/motor_command', Float32, queue_size = 10)
        self.brake_control_pub = rospy.Publisher('/brake_command', Bool, queue_size = 10)
        self.obs_msg = Bool()
        self.obs_msg.data = False
    
    #   Radar Call Backs :
    def lead_distance_callback(self,msg):                      
        self.lead_distance = msg.data # meters

    def lead_relative_velocity_callback(self, msg):            
        self.lead_relative_velocity = msg.data  # m/s

    def lead_valid_callback(self, msg):                     
        self.lead_valid = msg.data    
    
    # Control Callback
    def velocity_callback(self, msg):
        
        #   Computing ego states.      
        self.ego_pos += self.ego_vel * (rospy.Time.now() - self.cur_time).to_sec()               

        # Compute Lead Vehicle States Properly using RADAR DATA
        if self.lead_valid and self.lead_distance is not None:       

            self.obs_pos = self.ego_pos + self.lead_distance
            self.obs_vel = self.ego_vel + self.lead_relative_velocity        # need to confirm on this sign

            self.VLC.get_control_action(
                [self.ego_pos, self.ego_vel],
                [self.obs_pos, self.obs_vel]
            )

        else:                                                  
            if self.prev_lead_valid and not self.lead_valid:    #Safety Improvement
                rospy.logwarn("Lead vehicle lost — switching to cruise mode")
            
            self.VLC.get_control_action(
                [self.ego_pos, self.ego_vel],
                None
            )
        self.prev_lead_valid = self.lead_valid

        
        self.ego_vel = msg.data * 5 / 18                               # For converting the data to m/s from km/hr
        self.cur_time = rospy.Time.now()

        # Storing Ego vehicle states
        self.store_position.append(self.ego_pos)
        self.store_velocity.append(self.ego_vel)

        # Storing Lead Vehicle States Properly using RADAR DATA
        if self.lead_valid and self.lead_distance is not None:
            self.store_obs_pos.append(self.obs_pos)
            self.store_obs_vel.append(self.obs_vel)
            self.store_lead_distance.append(self.lead_distance)
            self.store_lead_rel_vel.append(self.lead_relative_velocity)
        else:
            self.store_obs_pos.append(None)
            self.store_obs_vel.append(None)
            self.store_lead_distance.append(None)
            self.store_lead_rel_vel.append(None)
        
        # Storing time
        self.store_time.append((self.cur_time - self.start_time).to_sec())

        # Printing data to the terminal
        rospy.loginfo(f'The Throttle command is : {self.VLC.throttle_pot} V')
        rospy.loginfo(f"Absolute Ego states (p,v,t):  {self.store_position[-1]},{self.store_velocity[-1]}, {self.store_time[-1]}")
        rospy.loginfo(f"Absolute Lead pos: {self.store_obs_pos[-1]}, Absolute lead vel : {self.store_obs_vel[-1]}")
        if self.store_obs_pos[-1] is not None:
            separation = self.store_obs_pos[-1] - self.store_position[-1]
            rospy.loginfo(f"Separation from lead vehicle : {separation}")
        
        # Safety supervisor
        if self.lead_valid and self.lead_distance is not None:
            if self.lead_distance >= CP.Dd:
                self.longitudinal_control_pub.publish(self.VLC.throttle_pot)
            else:
                self.emergency_brake()
        else:
            self.longitudinal_control_pub.publish(self.VLC.throttle_pot)

    def emergency_brake(self):
        
        rospy.logwarn('Emergency brake activated !!!!')
        self.VLC.throttle_pot = 0.0
        self.longitudinal_control_pub.publish(self.VLC.throttle_pot)

        
    def vehicle_shutdown_callback(self, event):
        rospy.loginfo(f'{self.store_position}')
        
        with open(self.file_path, 'w', newline='') as file:
            writer = csv.writer(file)

            writer.writerow(["Ego_Position(m)","Ego_Velocity(m/s)", "Obs_Position(m)", "Obs_Velocity(m/s)","Separation(m)", "Relative velocity(m/s)","Time(s)"])
            writer.writerows([[pos, vel, obs_pos, obs_vel,dis,rel_vel, time] for pos, vel, obs_pos, obs_vel,dis, rel_vel, time in zip(self.store_position, self.store_velocity, self.store_obs_pos, self.store_obs_vel,self.store_lead_distance, self.store_lead_rel_vel,  self.store_time)])  # Saves as columns
        rospy.loginfo('The test is over, vehicle is shutting down')
        rospy.signal_shutdown('Shutting down .....') 
        
        
       
if __name__ == '__main__':
    
    VC = Communication()
    VC.start_vehicle()


# Here Obstacle position is	Absolute world coordinate
# Separation is	Relative distance

# And mathematically:
## separation=obstacle_position−ego_position
