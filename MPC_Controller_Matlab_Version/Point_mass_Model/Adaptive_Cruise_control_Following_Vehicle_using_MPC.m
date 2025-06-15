clear;
close all;
clc;

%% MODEL PREDICTIVE CONTROLLER
% Simulating a scenario where the controller sees the lead vehicle at a distance of 35m
% sensor and systems reaction combined takes whole 3sec and
% after that vehicle starts decelerating as per requirement and starts following the lead vehicle.
% The simulation is done on point mass model. 


%% Parameters
v_lead = 5;                   % Lead vehicle speed (m/s)
dis_min = 1;                  % Minimum safe distance (m)
T = 0.1;                      % Sampling time (s)
N = 5;                        % Prediction horizon (5 seconds)
v_max_ego = 5.56;             % Max speed of ego vehicle (m/s)
v_min_ego = 0;                % Min speed of ego vehicle (m/s)
a_max = 1.4;                  % Max acceleration (m/s²)
a_min = -3.5;                 % Max deceleration (m/s²)
jerk_limit = 3;               % Max jerk (m/s³)
T_headway = 2;                % Time headway for safe distance (s)
d_detection = 100;             % Detection distance (m)
time_delay = 3;               % Reaction delay (s)

%% Initial Conditions
x_lead = 100;                 % Lead vehicle initial position (m)
x_ego = 0;                    % Ego vehicle initial position (m)
v_ego = 0;                    % Ego vehicle initial speed (m/s)
a_ego = 0;                    % Initial acceleration

%% Simulation Time
t_end = 400;                  % End time of simulation (s)
time = 0:T:t_end;
n = length(time);

%% Pre-allocation
x_ego_array = zeros(1, n);
v_ego_array = zeros(1, n);
a_ego_array = zeros(1, n);
x_lead_array = zeros(1, n);
d_array = zeros(1, n);

%% State-Space Model
A = [1, T; 0, 1];
B = [0.5*T^2; T];

%% MPC Weights
w_pos = 100;                  % Position weight
w_vel = 50;                   % Velocity weight
w_acc = 80;                    % Acceleration effort weight

%% Precompute Jerk Constraints (Between a_1 to a_N)
A_jerk = [];
for i = 1:N-1
    % Upper bound: a_{k+1} - a_k <= jerk_limit * T
    row_upper = zeros(1, N);
    row_upper(i) = -1;
    row_upper(i+1) = 1;
    A_jerk = [A_jerk; row_upper];
    
    % Lower bound: a_k - a_{k+1} <= jerk_limit * T
    row_lower = zeros(1, N);
    row_lower(i) = 1;
    row_lower(i+1) = -1;
    A_jerk = [A_jerk; row_lower];
end
b_jerk = jerk_limit * T * ones(2*(N-1), 1);

%% Simulation Loop
detected_time = -inf;  % Initialize detection time

for k = 1:n
    % Relative distance
    d_rel = x_lead - x_ego;
    d_array(k) = d_rel;

    % Store results
    x_ego_array(k) = x_ego;
    v_ego_array(k) = v_ego;
    a_ego_array(k) = a_ego;
    x_lead_array(k) = x_lead;

    % Safe distance calculation
    d_safe = dis_min + T_headway * v_ego;

    % 3-Second Delay Logic
    if d_rel < d_detection
        if detected_time == -inf
            detected_time = time(k); % Start timer on first detection
        end
        time_since_detection = time(k) - detected_time;
    else
        detected_time = -inf; % Reset if obstacle exits detection
        time_since_detection = 0;
    end

    % Reference state selection
    if (detected_time ~= -inf) && (time_since_detection >= time_delay)
        % Safe following mode (after delay)
        ref_pos = x_lead - d_safe;
        ref_vel = v_lead;
    else
        % Desired speed mode (during delay or no detection)
        ref_pos = x_lead - dis_min - T_headway * v_max_ego;
        ref_vel = v_max_ego;
    end
    ref = [ref_pos; ref_vel];
    X_ref = repmat(ref, N, 1);


    % Build prediction matrices (A_bar, B_bar)
    A_bar = zeros(2*N, 2);
    B_bar = zeros(2*N, N);
    for i = 1:N
        A_bar(2*i-1:2*i, :) = A^i;
        for j = 1:i
            B_bar(2*i-1:2*i, j) = A^(i-j) * B;
        end
    end

    % Dynamic Velocity Constraints (Recalculated Each Step)
 
    C_vel = kron(eye(N), [0 1]);  % Velocity selector matrix
    A_vel_upper = C_vel * B_bar;
    b_vel_upper = v_max_ego - C_vel * (A_bar * [x_ego; v_ego]);
    A_vel_lower = -C_vel * B_bar;
    b_vel_lower = v_min_ego + C_vel * (A_bar * [x_ego; v_ego]);
    
    % Combine all constraints
    A_ineq = [A_vel_upper; A_vel_lower; A_jerk];
    b_ineq = [b_vel_upper; b_vel_lower; b_jerk];


    % Cost function matrices
    Q = blkdiag(w_pos * eye(N), w_vel * eye(N));
    R = w_acc * eye(N);
    H = 2 * (B_bar' * Q * B_bar + R);
    f = 2 * (B_bar' * Q * (A_bar * [x_ego; v_ego] - X_ref));

   
    % Jerk Limit Between Current Acceleration and First Control Input

    lb = a_min * ones(N, 1);
    ub = a_max * ones(N, 1);
    
    % Constrain first acceleration to respect jerk limit from current a_ego
    lb(1) = max(a_min, a_ego - jerk_limit * T);
    ub(1) = min(a_max, a_ego + jerk_limit * T);
  

    % Solve MPC with all constraints
    options = optimoptions('quadprog', 'Display', 'none');
    U_opt = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], options);

    % Apply first control input
    a_ego = U_opt(1);

    % Update dynamics
    v_ego = max(v_min_ego, min(v_ego + a_ego * T, v_max_ego));
    x_ego = x_ego + v_ego * T + 0.5 * a_ego * T^2;

    % Update lead vehicle
    x_lead = x_lead + v_lead * T;
end

%% Plots
figure;
plot(time, d_array);
title('Distance to Lead Vehicle');
xlabel('Time (s)');
ylabel('Distance (m)');
grid on;

figure;
plot(time, v_ego_array, 'b', 'DisplayName', 'Ego Vehicle');
hold on;
plot(time, ones(1, n) * v_lead, 'r--', 'DisplayName', 'Lead Vehicle');
title('Vehicle Speeds');
xlabel('Time (s)');
ylabel('Speed (m/s)');
legend;
grid on;

figure;
plot(time, a_ego_array, 'b');
title('Vehicle Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s²)');
grid on;