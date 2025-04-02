clc;
clear all;
close all;

%% MODEL PREDICTIVE CONTROLLER - Following Vehicle Scenario
% Simulating a scenario where the controller sees the lead vehicle at a distance of 100m
% sensor and systems reaction combined takes whole 3sec and after that 
% vehicle starts decelerating as per requirement and starts following the lead vehicle at a safe distance.
% The simulation is done on linear System model. 

%% Parameters
v_lead = 5;                   % Lead vehicle/obstacle speed (m/s)
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
M =0.202;

%% Initial Conditions
x_lead = 150;                 % Lead vehicle/obstacle initial position (m)
x_ego = 0;                    % Ego vehicle initial position (m)
v_ego = 0;                    % Ego vehicle initial speed (m/s)
a_ego = 0;                    % Initial acceleration
R_vel = 0;
S_vel = 0;

%% Simulation Time
t_end = 400;                  % End time of simulation (s)
time = 0:T:t_end;
n = length(time);
detected_time = -inf;

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
w_pos = 150;                  % Position weight
w_vel = 80;                   % Velocity weight
w_acc = 80;                   % Acceleration effort weight

%% Precomputing Jerk Constraints (Between a_1 to a_N)
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
%% Main Simulation Loop
for k = 1:n
    % Current distance
    d = x_lead - x_ego;
    d_array(k) = d;
    
    % Store results
    v_ego_array(k) = v_ego;
    x_ego_array(k) = x_ego;
    a_ego_array(k) = a_ego;
    x_lead_array(k) = x_lead;
    R_vel_array(k) = R_vel; 
    
    % Detection and delay logic
    if d < d_detection
        if detected_time == -inf
            detected_time = time(k);
        end
        time_since_detection = time(k) - detected_time;
    else
        detected_time = -inf;
        time_since_detection = 0;
    end

    % Reference generation
    if (detected_time ~= -inf) && (time_since_detection >= time_delay)
        d_safe = dis_min + T_headway*v_ego;  % Time headway = 2s
        ref_pos = x_lead - d_safe;
        ref_vel = v_lead;
    else
        ref_pos = x_lead - dis_min - 2*v_max_ego; % Buffer distance
        ref_vel = v_max_ego;
    end
    ref = [ref_pos; ref_vel];
    X_ref = repmat(ref, N, 1);
    
    % Prediction matrices
    A_bar = zeros(2*N, 2);
    B_bar = zeros(2*N, N);
    for i = 1:N
        A_bar(2*i-1:2*i, :) = A^i;
        for j = 1:i
            B_bar(2*i-1:2*i, j) = A^(i-j)*B;
        end
    end

    % Cost function matrices
    Q = blkdiag(w_pos * eye(N), w_vel * eye(N));
    R = w_acc * eye(N);
    H = 2 * (B_bar' * Q * B_bar + R);
    f = 2 * (B_bar' * Q * (A_bar * [x_ego; v_ego] - X_ref));
    
    % Dynamic Velocity Constraints (Recalculated Each Step)
 
    C_vel = kron(eye(N), [0 1]);  % Velocity selector matrix
    A_vel_upper = C_vel * B_bar;
    b_vel_upper = v_max_ego - C_vel * (A_bar * [x_ego; v_ego]);
    A_vel_lower = -C_vel * B_bar;
    b_vel_lower = v_min_ego + C_vel * (A_bar * [x_ego; v_ego]);
    
    % Combine all constraints
    A_ineq = [A_vel_upper; A_vel_lower; A_jerk];
    b_ineq = [b_vel_upper; b_vel_lower; b_jerk];

    % Velocity-dependent acceleration bounds based on linear system model
    a_lb = max(a_min, -v_ego*M);       % Lower bound: a >= -v_ego*M
    a_ub = min(a_max, (v_max_ego - v_ego)*M); % Upper bound: a <= (v_max - v_ego)*M

    % Bounds with jerk limits
    lb = a_lb * ones(N, 1);
    ub = a_ub * ones(N, 1);

    % Constrain first acceleration to respect jerk limit from current a_ego
    lb(1) = max(a_lb, a_ego - jerk_limit*T);
    ub(1) = min(a_ub, a_ego + jerk_limit*T);

    % Solve MPC
    %options = optimoptions('quadprog', 'Display', 'off');
    %[U_opt, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, []);
    
    % Solve MPC with all constraints
    options = optimoptions('quadprog', 'Display', 'none');
    U_opt = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], options);

    % Apply first control input
    a_ego = U_opt(1);
    
    % Apply control input (clamp R_vel)
    R_vel = v_ego + a_ego/M; % Desired velocity
    %R_vel_clamped = max(0, min(R_vel, v_max_ego));

    x_ego = x_ego + v_ego*T + 0.5*M*(R_vel -v_ego)*T^2;  % Position update
    S_vel = v_ego + M*(R_vel - v_ego)*T; % Actual velocity update
    
    v_ego = S_vel;
    % Update lead vehicle
    x_lead = x_lead + v_lead*T;
    

end

%% Plots
figure;
plot(time, d_array);
title('Distance to Obstacle');
xlabel('Time (s)'); ylabel('Distance (m)'); grid on;

figure;
plot(time, v_ego_array, 'b', 'DisplayName', 'Ego');
hold on;
yline(v_max_ego, 'r--', 'Max Speed');
title('Velocity Profile');
xlabel('Time (s)'); ylabel('Speed (m/s)'); legend; grid on;

figure;
plot(time, a_ego_array, 'b');
title('Acceleration Profile');
xlabel('Time (s)'); ylabel('Acceleration (m/s²)'); grid on;

figure;
plot(time, R_vel_array, 'b', 'DisplayName', 'Ego');
hold on;
yline(v_max_ego, 'r--', 'Max Speed');
title('Required Velocity Profile');
xlabel('Time (s)'); ylabel('Speed (m/s)'); legend; grid on;