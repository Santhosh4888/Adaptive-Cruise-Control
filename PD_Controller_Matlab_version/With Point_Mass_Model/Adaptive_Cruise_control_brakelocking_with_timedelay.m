clc;
clear all;
close all;

% Simulation a scenario where the controller sees the obstacle at a distance of 30m
% sensor and systems reaction combined takes whole 3sec and
% after that vehicle starts decelerating at set max value.
% The simulation is done on point mass model
%% Vehicle is already at its top speed


% Initializing the values
Min_stop_distance = 1; % The value vehicle has to maintain from the object when it is at rest
C_dis = 68; % Initial distance in m
C_acc = 0;
C_vel = 5.56; % initial velocity in m/s
Kp = 0.05;
Kd = 2 * sqrt(Kp);
D_vel = 0; % Desired Velocity in m/s
T = 0.1;
p_dis = 68;
p_vel = 5.56;
P_acc = 0;
a_max = 1.4; % Max acceleration in m/s^2
d_max = -3.5; % Max deceleration in m/s^2
jerk_limit = 3;    % Maximum jerk of the vehicle (m/s3)

% Time delay settings
time_delay = 3; % 3 seconds delay considering all system dynamics
delay_steps = time_delay / T; % calculate number of steps for delay
step_count = 0; % initializing step counter

% Initializing arrays to store results
Time_1 = 0;
f_dis_1 = [68];
f_acc_1 = [0];
f_vel_1 = [5.56];
error_array = [32];
% simulation loop
while true
    E = (Min_stop_distance - p_dis); % calculate error
    E_vel = D_vel - p_vel; % calculate velocity error
    
    % simulation stop condition
    if p_dis <= Min_stop_distance || abs(E) < 0.001 || C_vel < 0
        C_acc = 0;
        C_vel = 0;
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
        

        % update velocity and distance
        C_vel = p_vel + (C_acc * T);
        C_dis = p_dis - (p_vel * T + 0.5 * C_acc * T^2);
     
    else 
        C_dis = p_dis - (p_vel * T + 0.5 * C_acc * T^2);
    end

    % Increase the step counter after every time step
    step_count = step_count + 1;

    % update previous values
    p_vel = C_vel;
    p_dis = C_dis;
    P_acc = C_acc;

    % store results
    f_dis_1 = [f_dis_1, C_dis];
    f_acc_1 = [f_acc_1, C_acc];
    f_vel_1 = [f_vel_1, C_vel];
    Time_1 = [Time_1, Time_1(end) + T];
    error_array= [error_array,E];
end

% plot results
figure;
plot(Time_1(1:end), f_acc_1);
xlabel('Time (s)');
ylabel('Desired Acceleration (m/s^2)');
title('Desired Acceleration over Time');
grid on;

figure;
plot(Time_1(1:end), f_dis_1);
xlabel('Time (s)');
ylabel('Current Distance (m)');
title('Current Distance over Time');
grid on;

figure;
plot(Time_1(1:end), f_vel_1);
xlabel('Time (s)');
ylabel('Current Velocity (m/s)');
title('Current Velocity over Time');
grid on;   