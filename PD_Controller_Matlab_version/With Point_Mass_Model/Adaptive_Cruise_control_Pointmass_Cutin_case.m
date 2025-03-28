% Following a Lead Vehicle Scenario which is moving with constant velocity 
% at a certain distance at speed lower the ego vehicle
% The car will be moving in speed mode until it finds the lead vehicle and 
% then transits into safe distance mode. 
%% Here PD controller is used to test the algorithm on point mass model in the cut-in scenario.
clc;
clear all;
close all;

% Parameters
v_lead = 5;             % Lead vehicle speed (m/s)
dis_min = 1;            % Desired safe distance (m)
Kp = 0.28;              % Proportional gain
Kd = 2 * sqrt(Kp);      % Derivative gain

% Initial conditions
v_ego = 5;         % Initial speed of following vehicle (m/s)
v_max_ego = 5.56;  % Maximum velocity of the ego vehicle
x_lead = 10;        % Initial position of lead vehicle (m)
x_ego = 0;         % Initial position of following vehicle (m)
dt = 0.1;          % Time step (s)
t_end = 500;       % Simulation end time (s)
a_max = 1.4;       % Maximum acceleration of the ego (m/s²)
d_max = -3.5;      % Maximum deceleration of the ego (m/s²)
jerk_limit = 3;    % Maximum jerk of the vehicle (m/s³)
a_current = 0;     % Current acceleration of the vehicle
d_detection = 100; % distance at which the lead object detected (m)

% Time delay settings
delay_counter = 0; % Counter to track the time delay
time_delay = 3;    % 3 seconds delay
delay_steps = time_delay / dt; % Number of steps for delay
d_safe = 0;

% Time array
time = 0:dt:t_end;
n = length(time);

% Preallocate arrays for distance, speed, and position
d_array_1 = zeros(1, n);
v_ego_array_1 = zeros(1, n);
x_ego_array_1 = zeros(1, n);
v_lead_array_1 = zeros(1, n);
x_lead_array_1 = zeros(1, n);
a_ego_array_1 = zeros(1, n);
error_array_1 = zeros(1,n);

%% Simulation loop
for i = 1:n
    % Calculate the current distance to the lead vehicle
    d = x_lead - x_ego;

    if d < d_detection %distance at which ego detects the lead vehicle
        % Time delay handling
        if delay_counter < delay_steps
            error = 0;  % Speed mode 
            error_dot = v_max_ego - v_ego;  % Rate of change of error
        else
            % Safe distance mode
            d_safe = dis_min + 2 * v_ego;
            error = d - d_safe; % (current distance - desired safe distance )
            error_dot = v_lead - v_ego;  
        end
        
        % PD control: calculate the desired acceleration
        a_ego = Kp * error + Kd * error_dot;

        % Apply jerk limit to acceleration
        if (a_ego - a_current) / dt > jerk_limit
            a_ego = a_current + jerk_limit * dt;
        elseif (a_ego - a_current) / dt < -jerk_limit
            a_ego = a_current - jerk_limit * dt;
        end
        
        % Limit acceleration and deceleration
        if a_ego > a_max
            a_ego = a_max;
        elseif a_ego < d_max
            a_ego = d_max;
        end
        
        % Update the speed of the ego vehicle
        v_ego = v_ego + a_ego * dt;

        % Limit the speed of the ego vehicle to the maximum speed
        if v_ego > v_max_ego
            v_ego = v_max_ego;
        end
        
        % Update positions of the vehicles
        x_lead = x_lead + v_lead * dt;
        x_ego = x_ego + v_ego * dt + 0.5 * a_ego * dt^2;
        
        % Store values
        d_array_1(i) = d;
        v_ego_array_1(i) = v_ego;
        x_ego_array_1(i) = x_ego;
        v_lead_array_1(i) = v_lead;
        x_lead_array_1(i) = x_lead;
        a_ego_array_1(i) = a_ego;
        error_array_1(i) = error;
            
        % Update the current acceleration
        a_current = a_ego;

        % Increment delay counter
        delay_counter = delay_counter + 1;
    else
        % Speed mode: the lead vehicle is far, so maintain max speed
        error = 0;
        error_dot = v_max_ego - v_ego;  % Rate of change of error
        a_ego = Kp * error + Kd * error_dot;  % PD control for speed mode

        % Apply jerk limit to acceleration
        if (a_ego - a_current) / dt > jerk_limit
            a_ego = a_current + jerk_limit * dt;
        elseif (a_ego - a_current) / dt < -jerk_limit
            a_ego = a_current - jerk_limit * dt;
        end
        
        % Limit acceleration and deceleration
        if a_ego > a_max
            a_ego = a_max;
        elseif a_ego < d_max
            a_ego = d_max;
        end

        % Update ego vehicle speed
        v_ego = v_ego + a_ego * dt;

        % Update positions
        x_lead = x_lead + v_lead * dt;
        x_ego = x_ego + v_ego * dt + 0.5 * a_ego * dt^2;

        % Store values
        d_array_1(i) = d;
        v_ego_array_1(i) = v_ego;
        x_ego_array_1(i) = x_ego;
        v_lead_array_1(i) = v_lead;
        x_lead_array_1(i) = x_lead;
        a_ego_array_1(i) = a_ego;
        error_array_1(i) = error;

        % Update current acceleration
        a_current = a_ego;
    end
end

% Plot results
figure;
plot(time, d_array_1);
title('Distance to Lead Vehicle');
xlabel('Time (s)');
ylabel('Distance (m)');
grid on;

figure;
plot(time, v_ego_array_1, 'b', 'DisplayName', 'Ego Vehicle');
hold on;
plot(time, v_lead_array_1, 'r', 'DisplayName', 'Lead Vehicle');
title('Vehicle Speeds');
xlabel('Time (s)');
ylabel('Speed (m/s)');
legend;
grid on;

figure;
plot(time, x_ego_array_1, 'b', 'DisplayName', 'Ego Vehicle');
hold on;
plot(time, x_lead_array_1, 'r', 'DisplayName', 'Lead Vehicle');
title('Vehicle Positions');
xlabel('Time (s)');
ylabel('Position (m)');
legend;
grid on;

figure;
plot(time, a_ego_array_1, 'b', 'DisplayName', 'Ego Vehicle');
title('Vehicle Accelerations');
xlabel('Time (s)');
ylabel('Acceleration (m/s²)');
legend;
grid on;
