#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Float32, Bool
import H_Common_Params as CP
import Longitudinal_Controller as LC
import csv
import os

class Communication:
    
    def __init__(self):
        
        self.ego_vel = 0.0                                                                       # Velocity of ego vehicle in m/s
        self.ego_pos = 0.0                                                                       # Position of ego vehicle in meters, estimated for now, once LIDAR sensor is ready, this will be obtained directly from it
        self.obs_vel = 0.0                                                                      # Velocity of obstacle vehicle in m/s, given for now, once RADAR sensor is ready, this will be obtained directly from it 
        #self.obs_vel = 0.0
        self.obs_pos = 100.0  
        self.Threshold = 90.0                                                                   # Position of obstacle vehicle in meters, given for now, once LIDAR sensor is ready, this will be obtained directly from it
        self.start_time = None
        self.cur_time = None
        self.VLC = LC.VLC()                                                                      # Getting the vehicle Longitudinal controller
        self.longitudinal_control_pub = None
        self.store_position = [self.ego_pos]
        self.store_velocity = [self.ego_vel]
        self.store_obs_pos = [self.obs_pos]
        self.store_obs_vel = [self.obs_vel]
        self.store_time = []
        self.save_dir = os.path.join(os.path.dirname(__file__), '..', 'data')
        self.file_path = os.path.join(self.save_dir, 'data_may_8_mpc_2_S3.csv')
        
    def start_vehicle(self):
        
        rospy.init_node('control_node', anonymous = True)
        print("Node Started")
        rospy.Timer(rospy.Duration(CP.H_total_experiment_time), self.vehicle_shutdown_callback, oneshot = True) # Sets the total experiment time
        self.start_time = rospy.Time.now()
        self.cur_time = rospy.Time.now()
        self.store_time.append((self.cur_time - self.start_time).to_sec())
        self.create_publishers()
        self.start_subscribers()
        rospy.spin()
            
    def start_subscribers(self):     
        
        rospy.Subscriber('/velocity_feedback', Float32, self.velocity_callback, queue_size = 10)
        
    def create_publishers(self):
        
        self.longitudinal_control_pub = rospy.Publisher('/motor_command', Float32, queue_size = 10)
        self.brake_control_pub = rospy.Publisher('/brake_command', Bool, queue_size = 10)
        self.obs_msg = Bool()
        self.obs_msg.data = False
        
    def velocity_callback(self, msg):
        
        self.ego_pos += self.ego_vel * (rospy.Time.now() - self.cur_time).to_sec()               # Estimating the separation travelled in the time at which the data is given
        self.obs_pos += self.obs_vel * (rospy.Time.now() - self.cur_time).to_sec()
        self.ego_vel = msg.data * 5 / 18                                                         # For converting the data to m/s from km/hr
        self.cur_time = rospy.Time.now()
        self.store_position.append(self.ego_pos)
        self.store_velocity.append(self.ego_vel)
        self.store_obs_pos.append(self.obs_pos)
        self.store_obs_vel.append(self.obs_vel)
        self.store_time.append((self.cur_time - self.start_time).to_sec())
        if self.obs_pos - self.ego_pos < self.Threshold:
            self.VLC.get_control_action([self.ego_pos, self.ego_vel], [self.obs_pos, self.obs_vel])
        else:
            self.VLC.get_control_action([self.ego_pos, self.ego_vel], None)

        rospy.loginfo(f'The control signal is : {self.VLC.throttle_pot} V')
        rospy.loginfo(f"data :  {self.store_position[-1]},{self.store_velocity[-1]}, {self.store_time[-1]}")
        rospy.loginfo(f"Obstacle position : {self.store_obs_pos[-1]}, Obstacle velocity : {self.store_obs_vel[-1]}")
        rospy.loginfo(f"separation : {self.store_obs_pos[-1] - self.store_position[-1]}")
        if self.obs_pos >= self.ego_pos + CP.Dd:
            self.longitudinal_control_pub.publish(self.VLC.throttle_pot)
        else:
            self.emergency_break()
            
        if self.ego_pos >= 10.0:                                             # For now it is assumed that, once the ego vehicle crosses 10 m, it detects the obstacle
            self.obs_msg.data = True
            self.brake_control_pub.publish(self.obs_msg.data)
        else:
            self.obs_msg.data = False
            self.brake_control_pub.publish(self.obs_msg.data)
        
    def vehicle_shutdown_callback(self, event):
        rospy.loginfo(f'{self.store_position}')
        
        with open(self.file_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Ego_Position(m)","Ego_Velocity(m/s)", "Obs_Position(m)", "Obs_Velocity(m)", "Time(s)"])
            writer.writerows([[pos, vel, obs_pos, obs_vel, time] for pos, vel, obs_pos, obs_vel, time in zip(self.store_position, self.store_velocity, self.store_obs_pos, self.store_obs_vel, self.store_time)])  # Saves as columns
        rospy.loginfo('The test is over, vehicle is shutting down')
        rospy.signal_shutdown('Shutting down .....') 
        
    def emergency_break(self):
        
        rospy.logwarn('Emergency brake activated !!!!')
        self.VLC.throttle_pot = 0.0
        self.longitudinal_control_pub.publish(self.VLC.throttle_pot)
        
if __name__ == '__main__':
    
    VC = Communication()
    VC.start_vehicle()