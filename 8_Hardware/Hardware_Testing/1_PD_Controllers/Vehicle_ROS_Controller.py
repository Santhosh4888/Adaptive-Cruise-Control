import numpy as np
import matplotlib.pyplot as plt
import Controller
import sys
import os
import rospy
from std_msgs.msg import Float32
internal_HPDC_dir = os.path.dirname(os.path.abspath(__file__))
internal_HT_dir = os.path.abspath(os.path.join(internal_HPDC_dir, '..'))
internal_H_dir = os.path.abspath(os.path.join(internal_HT_dir, '..'))
root_dir = os.path.abspath(os.path.join(internal_H_dir, '..'))
internal_CP_dir = os.path.abspath(os.path.join(root_dir, '6_Common_Paramaters'))
sys.path.append(internal_CP_dir)
import Common_Params as CP

class Vehicle_Controller:
    
    def __init__(self, controller):
        
        self.controller = controller
        self.ego_values = [0.0, 0.0]
        self.preceeding_values = [75.0, 0.0]
        self.throttle_pot = 0.0
        self.cur_time = 0.0
        
    def start_subscribers(self):
        
        rospy.Subscriber('velocity_feedback', Float32, self.velocity_callback)
        #rospy.Subscriber('rpm_feedback', Float32, self.rpm_callback)
        
    def start_vehicle(self):
        
        rospy.init_node('control_node')
        rospy.loginfo(f'Starting the vehicle')
        self.control_pub = rospy.Publisher('motor_command', Float32, queue_size = 10)
        self.cur_time = rospy.Time.now()
        self.cur_exp_time = rospy.Time.now()
        self.start_subscribers()
        
    def rpm_to_velocity(self, rpm):                                                    # RPM should be in rpm
        
        return rpm * 2 * np.pi * CP.tyre_radius / (CP.gear_ratio * 60)

    def velocity_to_rpm(self, velocity):                                               # Velocity should be in m/s
        
        return velocity * 60 * CP.gear_ratio / (2 * np.pi * CP.tyre_radius)
    
    def get_motor_rpm(self, throttle_cmd):
        
        return max(0, min(CP.max_rpm, CP.slope_rpm_tcmd * (throttle_cmd - 0)))

    def get_throttle_cmd(self, motor_rpm):
        
        return max(0, min(CP.max_tcmd, 1 / CP.slope_rpm_tcmd * (motor_rpm - 0)))
    
    def get_control_action(self):
        
        a = self.controller.get_acceleration(self.ego_values, self.preceeding_values)            # Getting control action from the controller
        req_vel = max(0.0, min(CP.ego_max_v, self.ego_values[1] + a / CP.A))                     # Calculating required velocity from the acceleration
        throttle_cmd = self.get_throttle_cmd(self.velocity_to_rpm(req_vel))                      # Calculating throttle command from required velocity
        if throttle_cmd <= CP.threshold_throttle_cmd:                                            # Restricting throttle command below 5 percent to zero
            throttle_cmd = 0.0
        self.throttle_pot = CP.rbf_model_tcmd_to_requested_pot(np.array([throttle_cmd]))[0]      # Calculating the required throttle pot from throttle command 
        rospy.loginfo(f'{self.throttle_pot}')                                                    # Logging the throttle pot data
    
    def control_vehicle(self):
        
        self.get_control_action()
        while self.cur_exp_time <= CP.H_total_experiment_time:
            self.control_pub.publish(self.throttle_pot)                                          # Publishing the throttle pot onto the topic motor_command
            if (rospy.Time.now() - self.cur_exp_time).to_sec() >= 0.5:
                self.cur_exp_time = rospy.Time.now()
                self.get_control_action()
        rospy.loginfo(f'The experiment ended')
        self.emergency_break()
        rospy.signal_shutdown('Shutting down')
        
    def emergency_break(self):
        
        while self.ego_values[1] > 0:
            self.throttle_pot = 0.0
            self.control_pub.publish(self.throttle_pot)
            self.cur_exp_time = rospy.Time.now()
            while (rospy.Time.now() - self.cur_exp_time).to_sec() <= 0.5:
                continue
        
    def velocity_callback(self, msg):
        
        self.ego_values[0] += self.ego_values[1] * (rospy.Time.now() - self.cur_time).to_sec()   # Estimating the separation travelled in the time at which the data is given
        self.ego_values[1] = msg.data * 5 / 18                                                   # For converting the data to m/s from km/hr
        self.cur_time = rospy.Time.now()                                    
        
        
        
        
        

