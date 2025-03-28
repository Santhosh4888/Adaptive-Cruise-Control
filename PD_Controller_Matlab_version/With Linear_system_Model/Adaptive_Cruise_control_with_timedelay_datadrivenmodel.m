clc;
clear all;
close all;

% Simulation a scenario where the controller sees the obstacle at a distance of 35m
% sensor and systems reaction combined takes whole 3sec and
% after that vehicle starts decelerating at set max value.
% The simulation is done on data driven model
%% Here in this code, vehicle is assumed to be already at its Top speed


% Initializing the values
Min_stop_distance = 1; % The value vehicle has to maintain from the object when it is at rest
V_max = 5.56; % Maximum velocity of the vehicle
C_dis = 68; % Initial distance in m
C_acc = 0;
C_Vel = 5.56; % initial velocity in m/s (=20kmph)
Kp = 0.5;
Kd = 2 * sqrt(Kp);
D_vel = 0; % Desired Velocity in m/s
T = 0.1;
p_dis = 68;
S_Vel = 5.56; % 20kmph
P_acc = 0;
a_max = 1.4; % Max acceleration in m/s^2
d_max = -3.5; % Max deceleration in m/s^2
jerk_limit = 3;    % Maximum jerk of the vehicle (m/s3)
M = 0.11 ; % Constant capturing system dynamics % It is too small to handle (1/s)
R_Vel = 0; %Initial Velocity

% Time delay settings
time_delay = 3; % 3 seconds delay considering all system dynamics
delay_steps = time_delay / T; % calculate number of steps for delay
step_count = 0; % initializing step counter

%Non-linear eqn constants
A = 1.096;    % Parameter A
B = 0.448;    % Parameter B
C = -1.496*10^-5;    % Parameter C
D = -1.044;    % Parameter D
E = 0.972;    % Parameter E

% Initializing arrays to store results

Time_2 = 0;
f_dis_2 = [68];
f_acc_2 = [0];
f_vel_2 = [5.56];
R_Vel_1 = [0];
error_array = [];
% simulation loop
while true
    E = Min_stop_distance - p_dis; % calculate error % need to look into this eqn
    E_vel = D_vel - C_Vel; % calculate velocity error
    
    % simulation stop condition
    if p_dis < Min_stop_distance || abs(E) < 0.01 || C_Vel < 0
        C_acc = 0;
        C_Vel = 0;
        C_dis = p_dis;
        break;
    end

    % Implement the time delay
    if step_count >= delay_steps
        
        C_acc = E * Kp + E_vel * Kd; %Controlled acceleration input

        if (C_acc-P_acc)/T > jerk_limit
            C_acc = P_acc + jerk_limit * T ;
        elseif (C_acc-P_acc)/T < -jerk_limit
             C_acc = P_acc - jerk_limit * T ;
        end

        if C_acc > a_max
            C_acc = a_max;
        elseif C_acc < d_max
            C_acc = d_max;
        end
        
        R_Vel = C_Vel + (C_acc/M); %System Model

        if R_Vel < 0
            R_Vel = 0;
        elseif R_Vel > 5.56
           R_Vel = 5.56;
        end 

        %t_cmd = (R_Vel * 100)/V_max
        
       
        % update velocity and distance
        %Current_rpm = p_vel + (C_acc * T);
        S_Vel = C_Vel + M*(R_Vel - C_Vel)*T;   %System Model
        %S_Vel = A * C_Vel + B * t_cmd * (1 / (1 + exp(1000 * (-R_Vel + C_Vel)))) + (C * C_Vel^2 + D * C_Vel + E) * T;

        %Vel = System_rpm / 432; %Velocity Conversion

        C_dis = p_dis - ((C_Vel * T) + (0.5 * M * (R_Vel-C_Vel)* T^2) );
     
    else 
        C_dis = p_dis - ((C_Vel * T) + (0.5 * M * (R_Vel-C_Vel)* T^2) );
    end

    % Increase the step counter after every time step
    step_count = step_count + 1;

    % update previous values
    C_Vel = S_Vel;
    p_dis = C_dis;
    P_acc = C_acc;

    % store results
    f_dis_2 = [f_dis_2, C_dis];
    f_acc_2 = [f_acc_2, C_acc];
    f_vel_2 = [f_vel_2, S_Vel];
    Time_2 = [Time_2, Time_2(end) + T];
    error_array= [error_array,E];
    R_Vel_1 = [R_Vel_1 , R_Vel];

end

% plot results
figure;
plot(Time_2(1:end), f_acc_2);
xlabel('Time (s)');
ylabel('Desired Acceleration (m/s^2)');
title('Desired Acceleration over Time');
grid on;

figure;
plot(Time_2(1:end), f_dis_2);
xlabel('Time (s)');
ylabel('Current Distance (m)');
title('Current Distance over Time');
grid on;

figure;
plot(Time_2(1:end), f_vel_2);
xlabel('Time (s)');
ylabel('Current Velocity (m/s)');
title('Current Velocity over Time');
grid on;   