import numpy as np
import sys
import os
internal_WSM_dir = os.path.dirname(os.path.abspath(__file__))
internal_PDC_dir = os.path.abspath(os.path.join(internal_WSM_dir, '..'))
root_dir = os.path.abspath(os.path.join(internal_PDC_dir, '..'))
internal_CP_dir = os.path.abspath(os.path.join(root_dir, '6_Common_Paramaters'))
sys.path.append(internal_CP_dir)
import Common_Params as CP
import torch
import torch.nn as nn

class System(nn.Module):
    
    def __init__(self, input_size = CP.input_size, hidden_size_1 = CP.hidden_size_1, hidden_size_2 = CP.hidden_size_2, hidden_size_3 = CP.hidden_size_3, hidden_size_4 = CP.hidden_size_4, num_layers_1 = CP.num_layers_1, num_layers_2 = CP.num_layers_2, output_size = CP.output_size):
        super(System, self).__init__()
        
        self.cnn = nn.Sequential(
            nn.Conv1d(in_channels = input_size, out_channels = 16, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size = 2),
            nn.Conv1d(in_channels = 16, out_channels = 32, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size = 2)
        )
        
        self.lstm_1 = nn.LSTM(32, hidden_size_1, num_layers_1, batch_first = True)
        self.lstm_2 = nn.LSTM(hidden_size_1, hidden_size_2, num_layers_2, batch_first = True)
        self.fc_1 = nn.Linear(hidden_size_2, hidden_size_3)
        self.fc_2 = nn.Linear(hidden_size_3, output_size)
        self.RELU = nn.ReLU()

    def rpm_to_velocity(self, rpm):                                                    # RPM should be in rpm
        return rpm * 2 * np.pi * CP.tyre_radius / (CP.gear_ratio * 60)

    def velocity_to_rpm(self, velocity):                                               # Velocity should be in m/s
        return velocity * 60 * CP.gear_ratio / (2 * np.pi * CP.tyre_radius)
    
    def get_motor_rpm(self, throttle_cmd):
        return max(0, min(CP.max_rpm, CP.slope_rpm_tcmd * (throttle_cmd - 0)))

    def get_throttle_cmd(self, motor_rpm):
        return max(0, min(CP.max_tcmd, 1 / CP.slope_rpm_tcmd * (motor_rpm - 0)))
    
    def reset_seq(self):
        self.scaled_input = CP.scaler.transform([[0.0, 0.0, 0.0, - CP.VW * 9.81 * CP.velocity_to_rpm_ratio * CP.periodic_step / CP.VW, 0.0, 0.0, 0.0]])[0]
        self.input_seq = torch.FloatTensor([[[self.scaled_input[k] for k in range(CP.input_size)] for _ in range(CP.seq_length)]])
    
    def get_nn_input(self, throttle_cmd, motor_rpm):
        if motor_rpm == 0 and throttle_cmd != 0:
            scaled_input = CP.scaler.transform([[motor_rpm, (self.get_motor_rpm(throttle_cmd) - motor_rpm) * CP.periodic_step, (CP.velocity_to_rpm_ratio ** 2) * CP.periodic_step * CP.te * 2 * np.pi * CP.MAX_motor_torque / (CP.VW * 60), - CP.VW * 9.81 * CP.velocity_to_rpm_ratio * CP.periodic_step / CP.VW, - motor_rpm * CP.periodic_step / CP.VW, - self.rpm_to_velocity(motor_rpm) * motor_rpm * CP.periodic_step / CP.VW, 0.0]])[0]
        elif throttle_cmd == 0:
            scaled_input = CP.scaler.transform([[motor_rpm, (self.get_motor_rpm(throttle_cmd) - motor_rpm) * CP.periodic_step, 0.0, - CP.VW * 9.81 * CP.velocity_to_rpm_ratio * CP.periodic_step / CP.VW, - motor_rpm * CP.periodic_step / CP.VW, - self.rpm_to_velocity(motor_rpm) * motor_rpm * CP.periodic_step / CP.VW, 0.0]])[0]
        else:    
            scaled_input = CP.scaler.transform([[motor_rpm, (self.get_motor_rpm(throttle_cmd) - motor_rpm) * CP.periodic_step, (CP.velocity_to_rpm_ratio ** 2) * CP.periodic_step * CP.te * min(CP.MAX_motor_power / motor_rpm, 2 * np.pi * CP.MAX_motor_torque / 60) / CP.VW, - CP.VW * 9.81 * CP.velocity_to_rpm_ratio * CP.periodic_step / CP.VW, - motor_rpm * CP.periodic_step / CP.VW, - self.rpm_to_velocity(motor_rpm) * motor_rpm * CP.periodic_step / CP.VW, 0.0]])[0]
        new_input = torch.FloatTensor([[[scaled_input[k] for k in range(CP.input_size)]]])                               # (1, 1, feature_size)
        self.input_seq = torch.cat((self.input_seq[:, 1:, :], new_input), dim = 1)                                       # Shift left and append new value
        
    def update_seq(self, throttle_cmd, motor_rpm):
        if motor_rpm == 0 and throttle_cmd != 0:
            scaled_input = CP.scaler.transform([[motor_rpm, (self.get_motor_rpm(throttle_cmd) - motor_rpm) * CP.periodic_step, (CP.velocity_to_rpm_ratio ** 2) * CP.periodic_step * CP.te * 2 * np.pi * CP.MAX_motor_torque / (CP.VW * 60), - CP.VW * 9.81 * CP.velocity_to_rpm_ratio * CP.periodic_step / CP.VW, - motor_rpm * CP.periodic_step / CP.VW, - self.rpm_to_velocity(motor_rpm) * motor_rpm * CP.periodic_step / CP.VW, 0.0]])[0]
        elif throttle_cmd == 0:
            scaled_input = CP.scaler.transform([[motor_rpm, (self.get_motor_rpm(throttle_cmd) - motor_rpm) * CP.periodic_step, 0.0, - CP.VW * 9.81 * CP.velocity_to_rpm_ratio * CP.periodic_step / CP.VW, - motor_rpm * CP.periodic_step / CP.VW, - self.rpm_to_velocity(motor_rpm) * motor_rpm * CP.periodic_step / CP.VW, 0.0]])[0]
        else:    
            scaled_input = CP.scaler.transform([[motor_rpm, (self.get_motor_rpm(throttle_cmd) - motor_rpm) * CP.periodic_step, (CP.velocity_to_rpm_ratio ** 2) * CP.periodic_step * CP.te * min(CP.MAX_motor_power / motor_rpm, 2 * np.pi * CP.MAX_motor_torque / 60) / CP.VW, - CP.VW * 9.81 * CP.velocity_to_rpm_ratio * CP.periodic_step / CP.VW, - motor_rpm * CP.periodic_step / CP.VW, - self.rpm_to_velocity(motor_rpm) * motor_rpm * CP.periodic_step / CP.VW, 0.0]])[0]
        new_input = torch.FloatTensor([[[scaled_input[k] for k in range(CP.input_size)]]])                               # (1, 1, feature_size)
        self.input_seq = torch.cat((self.input_seq[:, : -1, :], new_input), dim = 1)                                     # Shift left and append new value
                
    def forward(self, x):
        
        x_init = x.clone()
        x = x.permute(0, 2, 1)                                                                                 # To convert it to the (batch_size, feature_size, sequence_lenght) required form for conv1d
        x = self.cnn(x)
        x = x.permute(0, 2, 1)                                                                                 # Converting back to (batch_size, sequence_length, feature_size)
        inter_lstm_out, _ = self.lstm_1(x)
        lstm_out, _ = self.lstm_2(inter_lstm_out)
        x = self.RELU(self.fc_1(lstm_out[:, -1, :]))
        prev_rpm = x_init[:, -1, 0].unsqueeze(-1)
        final_pred = torch.clamp(self.fc_2(x), min = CP.a_min * CP.periodic_step * CP.velocity_to_rpm_ratio + prev_rpm, max = CP.a_max * CP.periodic_step * CP.velocity_to_rpm_ratio + prev_rpm)
        return torch.clamp(final_pred, min = 0, max = CP.max_rpm)