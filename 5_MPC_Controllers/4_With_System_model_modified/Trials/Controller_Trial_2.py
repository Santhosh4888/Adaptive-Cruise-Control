import numpy as np
import cvxpy as cp 
import sys
import System_Model as SM
CP_module_dir = r'D:\lord_of_darkness\IITM\5th_year\DDP_Final\6_Common_Paramaters'
sys.path.append(CP_module_dir)
import Common_Params as CP  

class MPC:
    
    def __init__(self, Np = 12, Nc = 10):
        
        self.Np = Np                                         # Prediction horizon
        self.Nc = Nc                                         # Control horizon
        self.state_dim = 2                                   # State consists of seperation and velocity of ego vehicle
        self.control_dim = 1                                 # Control just has the acceleration of the ego vehicle
        self.desired_speed = CP.ego_max_v                    # Obstacle velocity of the vehicle
        self.a_curr = 0.0                                    # Current acceleration of the vehicle
        self.sim_sys_model = SM.System()
        self.sim_sys_model.load_state_dict(CP.non_linear_weights)
        self.sim_sys_model.reset_seq()
            
    def get_acceleration(self, ego_values, preceeding_values = None):
        
        ego_pos, ego_vel = ego_values
        if preceeding_values:
            obs_pos, obs_vel = preceeding_values
            #self.desired_speed = max(0.0, min(CP.stopping_velocities_distances_rbf(np.array([obs_pos - ego_pos]) / 1.5)[0], CP.ego_max_v))
        else:
            obs_pos, obs_vel = None, None
        
        Q = np.eye(2)                                                     # For defining a quadratic cost on the state variables
        R = np.eye(1)                                                     # For defining a quadratic cost on the control action
        P = np.eye(1) * 10000                                             # For defining a quadratic cost on the relaxation term
        x = cp.Variable((self.state_dim, self.Np + 1))                    # Np + 1 for handling Np steps in prediction horizon and 1 for initial state
        u = cp.Variable((self.control_dim, self.Np))                      # Control action for Np steps
        delta = cp.Variable((self.control_dim, self.Np))                  # Np relaxation terms for relaxing the CBF constraints
        
        if obs_pos:                                                       # If obstacle is there then CBF constraint is an essential constraint
            x0 = np.array([ego_pos, ego_vel])                             # Define the current state (initial state)
            cur_obs_pos = obs_pos
            cost = 0
            constraints = []
            for k in range(self.Np):
                x_ref = np.array([0, self.desired_speed])
                S = np.array([[0, 0], [0, 1]])
                
                cost += cp.quad_form(x_ref - S @ x[:, k], Q)              # Defining the cost function only for the vehicle to reach the set speed
                
                if k <= self.Nc: 
                    cost += cp.quad_form(u[:, k], R)
                cost += cp.quad_form(delta[:, k], P)
                
                constraints += [x[0, k + 1] == x[0, k] + (x[1, k] * CP.sample_time + 0.5 * u[0, k] * CP.sample_time ** 2)]
                constraints += [x[1, k + 1] == x[1, k] + u[0, k] * CP.sample_time]
                
                if k >= self.Nc:                                          # This is done to make all the control actions after Control horizon same as control action at the time instant of control horizon
                    constraints += [u[:, k] == u[:, k - 1]]
                
                constraints += [CP.a_min <= u[0, k], u[0, k] <= CP.a_max] # Constraints on acceleration
                
                constraints += [0.0 <= u[0, k] / CP.A + x[1, k], u[0, k] / CP.A + x[1, k] <= CP.ego_max_v] # Constraints on the acceleration based on the system model
                
                constraints += [0.0 <= x[1, k], x[1, k] <= CP.ego_max_v]  # Constraints on state of the vehicle
                constraints += [cur_obs_pos - 0.5 >= x[0, k]]             # Constraints on state of the vehicle
                
                # CBF constraint for obstacle collision avoidance
                constraints += [obs_vel - CP.Td * u[0, k] - x[1, k] >= - 0.1 * (obs_pos - CP.Dd - CP.Td * x[1, k] - x[0, k]) - delta[0, k]] 
                constraints += [delta[0, k] >= 0]
                
                if k == 0:
                    constraints += [CP.dec_Jerk_limit <= (u[0, k] - self.a_curr) / CP.sample_time, (u[0, k] - self.a_curr) / CP.sample_time <= CP.acc_Jerk_limit]
                else:
                    constraints += [CP.dec_Jerk_limit <= (u[0, k] - u[0, k - 1]) / CP.sample_time, (u[0, k] - u[0, k - 1]) / CP.sample_time <= CP.acc_Jerk_limit]
                    
                cur_obs_pos += obs_vel * CP.sample_time                   # Updating the obstacles position assuming it is moving at a constant velocity
                    
            constraints += [x[:, 0] == x0]
            constraints += [0.0 <= x[1, self.Np], x[1, self.Np] <= CP.ego_max_v]
            constraints += [cur_obs_pos - 0.5 >= x[0, self.Np]]
            
        else:
            x0 = np.array([0, ego_vel])                                   # Define the current state (initial state)
            cost = 0
            constraints = []
            for k in range(self.Np):
                x_ref = np.array([0, self.desired_speed])
                
                cost += cp.quad_form(x_ref - x[:, k], Q)                  # Defining the cost function
                
                if k <= self.Nc: 
                    cost += cp.quad_form(u[:, k], R)
                
                constraints += [x[0, k + 1] == 0]
                constraints += [x[1, k + 1] == x[1, k] + u[0, k] * CP.sample_time]
            
                if k >= self.Nc:
                    constraints += [u[:, k] == u[:, k - 1]]
                
                constraints += [CP.a_min <= u[0, k], u[0, k] <= CP.a_max] # Constraints on acceleration
                
                constraints += [0.0 <= u[0, k] / CP.A + x[1, k], u[0, k] / CP.A + x[1, k] <= CP.ego_max_v] # Constraints on the acceleration based on the system model
                
                constraints += [0.0 <= x[1, k], x[1, k] <= CP.ego_max_v]  # Constraints on state of the vehicle

                if k == 0:
                    constraints += [CP.dec_Jerk_limit <= (u[0, k] - self.a_curr) / CP.sample_time, (u[0, k] - self.a_curr) / CP.sample_time <= CP.acc_Jerk_limit]
                else:
                    constraints += [CP.dec_Jerk_limit <= (u[0, k] - u[0, k - 1]) / CP.sample_time, (u[0, k] - u[0, k - 1]) / CP.sample_time <= CP.acc_Jerk_limit]
            
            constraints += [x[:, 0] == x0]
            constraints += [0.0 <= x[1, self.Np], x[1, self.Np] <= CP.ego_max_v]
        
        problem = cp.Problem(cp.Minimize(cost), constraints)             # Solving for the optimization problem
        problem.solve(solver = cp.OSQP, verbose = False, max_iter = 50000)
    
        pred_a = u[0, 0].value
        """
        if pred_a is None:
            print('Controller failed')
            
        
        
        req_vel = ego_vel + pred_a / CP.A
        req_vel = max(0.0, min(req_vel, CP.ego_max_v))
        mpc_vel = ego_vel + CP.A * (req_vel - ego_vel) * CP.periodic_step
        
        count = 0
        threshold = 0.5 * 5 / 18
        while count < 2:

            cur_rpm = self.sim_sys_model.velocity_to_rpm(ego_vel)
            
            req_vel = ego_vel + pred_a / CP.A
            req_vel = max(0.0, min(req_vel, CP.ego_max_v))
            throttle_cmd = self.sim_sys_model.get_throttle_cmd(self.sim_sys_model.velocity_to_rpm(req_vel))
            if throttle_cmd <= CP.threshold_throttle_cmd:
                throttle_cmd = 0.0
            
            if count == 0:
                self.sim_sys_model.get_nn_input(throttle_cmd, cur_rpm)                            # Converts the current motor rpm and throttle command into a form suitable for neural network
            else:
                self.sim_sys_model.update_seq(throttle_cmd, cur_rpm)
                
            pred_rpm = self.sim_sys_model(self.sim_sys_model.input_seq)                           # Gets the prediction of neural network
            fin_rpm = CP.scaler.inverse_transform([[0, 0, 0, 0, 0, 0, pred_rpm.item()]])[0, -1]   # Descales the prediction of the neural network and gets the final output rpm
            fin_vel = self.sim_sys_model.rpm_to_velocity(fin_rpm)

            pred_a = CP.safety_factor * pred_a + (1 - CP.safety_factor) * (mpc_vel - fin_vel)
            
            pred_a = max(pred_a + CP.dec_Jerk_limit * CP.sample_time, min(pred_a + CP.acc_Jerk_limit * CP.sample_time, pred_a))
            pred_a = max(CP.a_min, min(pred_a, CP.a_max))   
            count += 1

        cur_rpm = self.sim_sys_model.velocity_to_rpm(ego_vel)
        
        req_vel = ego_vel + pred_a / CP.A
        req_vel = max(0.0, min(req_vel, CP.ego_max_v))
        throttle_cmd = self.sim_sys_model.get_throttle_cmd(self.sim_sys_model.velocity_to_rpm(req_vel))
        if throttle_cmd <= CP.threshold_throttle_cmd:
            throttle_cmd = 0.0
        
        self.sim_sys_model.update_seq(throttle_cmd, cur_rpm)                                    # Converts the current motor rpm and throttle command into a form suitable for neural network
        """            
        self.a_curr = pred_a
        return self.a_curr
                
            
            
            
          
        