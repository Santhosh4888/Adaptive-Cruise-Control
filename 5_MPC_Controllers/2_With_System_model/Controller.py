import numpy as np
import cvxpy as cp 
import sys
import System_Model as SM
import os
internal_WSM_dir = os.path.dirname(os.path.abspath(__file__))
internal_MPC_dir = os.path.abspath(os.path.join(internal_WSM_dir, '..'))
root_dir = os.path.abspath(os.path.join(internal_MPC_dir, '..'))
internal_CP_dir = os.path.abspath(os.path.join(root_dir, '6_Common_Paramaters'))
sys.path.append(internal_CP_dir)
import Common_Params as CP  

class MPC:
    
    def __init__(self, Np = 6, Nc = 4):
        
        self.Np = Np                                         # Prediction horizon
        self.Nc = Nc                                         # Control horizon
        self.state_dim = 2                                   # State consists of seperation and velocity of ego vehicle
        self.control_dim = 1                                 # Control just has the acceleration of the ego vehicle
        self.desired_speed = CP.ego_max_v                    # Obstacle velocity of the vehicle
        self.a_curr = 0.0                                    # Current acceleration of the vehicle
            
    def get_acceleration(self, ego_values, preceeding_values = None):
        
        ego_pos, ego_vel = ego_values
        if preceeding_values:
            obs_pos, obs_vel = preceeding_values
        else:
            obs_pos, obs_vel = None, None
        
        Q = np.eye(2)                                                     # For defining a quadratic cost on the state variables
        R = np.eye(1)                                                     # For defining a quadratic cost on the control action
        P = np.eye(1) * 10000                                             # For defining a quadratic cost on the relaxation term
        T = np.eye(1) * 100
        x = cp.Variable((self.state_dim, self.Np + 1))                    # Np + 1 for handling Np steps in prediction horizon and 1 for initial state
        u = cp.Variable((self.control_dim, self.Np))                      # Control action for Np steps
        alpha = cp.Variable((self.control_dim, self.Np))
        delta = cp.Variable((self.control_dim, self.Np))                  # Np relaxation terms for relaxing the CBF constraints
        
        M = 100000
        dt1 = CP.sample_time
        dt2 = 30 * CP.sample_time

        
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
                cost += cp.quad_form(alpha[:, k], T)
                
                expr = (u[0, k] / CP.A + x[1, k])

                constraints += [alpha[0, k] >= 0.0, alpha[0, k] <= 1.0]
                constraints += [expr <=  CP.threshold_velocity + M * (1 - alpha[0, k]), expr >= CP.threshold_velocity - M * alpha[0, k]]
                
                x_k1, v_k1 = cp.Variable(), cp.Variable()
                x_update1 = x[0, k] + x[1, k] * dt1 + 0.5 * u[0, k] * dt1 ** 2
                x_update2 = x[0, k] + x[1, k] * dt2 + 0.5 * u[0, k] * dt2 ** 2
                v_update1 = x[1, k] + u[0, k] * dt1
                v_update2 = x[1, k] + u[0, k] * dt2
                
                
                
                #constraints += [x_k1 - x_update1 <= M * alpha[0, k], x_k1 - x_update1 >= - M * alpha[0, k], x_k1 - x_update2 <= M * (1 - alpha[0, k]), x_k1 - x_update2 >= - M * (1 - alpha[0, k]), x[0, k + 1] == x_k1]
                #constraints += [v_k1 - v_update1 <= M * alpha[0, k], v_k1 - v_update1 >= - M * alpha[0, k], v_k1 - v_update2 <= M * (1 - alpha[0, k]), v_k1 - v_update2 >= - M * (1 - alpha[0, k]), x[1, k + 1] == v_k1]
                
                if k >= self.Nc:                                          # This is done to make all the control actions after Control horizon same as control action at the time instant of control horizon
                    constraints += [u[:, k] == u[:, k - 1]]
                
                constraints += [CP.a_min <= u[0, k], u[0, k] <= CP.a_max] # Constraints on acceleration
                
                constraints += [0.0 <= u[0, k] / CP.A + x[1, k], u[0, k] / CP.A + x[1, k] <= CP.ego_max_v] # Constraints on the acceleration based on the system model
                
                constraints += [0.0 <= x[1, k], x[1, k] <= CP.ego_max_v]  # Constraints on state of the vehicle
                constraints += [cur_obs_pos - 0.5 >= x[0, k]]             # Constraints on state of the vehicle
                
                # CBF constraint for obstacle collision avoidance
                constraints += [obs_vel - CP.Td * u[0, k] - x[1, k] >= - 0.1 * (cur_obs_pos - CP.Dd - CP.Td * x[1, k] - x[0, k]) - delta[0, k]] 
                constraints += [delta[0, k] >= 0]
                
                if k == 0:
                    constraints += [CP.dec_Jerk_limit * (dt1 * (1 - alpha[0, k]) + dt2 * alpha[0, k]) <= (u[0, k] - self.a_curr), (u[0, k] - self.a_curr) <= CP.acc_Jerk_limit * (dt1 * (1 - alpha[0, k]) + dt2 * alpha[0, k])]
                else:
                    constraints += [CP.dec_Jerk_limit * (dt1 * (1 - alpha[0, k]) + dt2 * alpha[0, k]) <= (u[0, k] - u[0, k - 1]), (u[0, k] - u[0, k - 1]) <= CP.acc_Jerk_limit * (dt1 * (1 - alpha[0, k]) + dt2 * alpha[0, k])]
                    
                cur_obs_pos += obs_vel * (dt1 * (1 - alpha[0, k]) + dt2 * alpha[0, k]) # Updating the obstacles position assuming it is moving at a constant velocity
                    
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
        problem.solve(solver = cp.HIGHS, verbose = False)
        self.a_curr = u[0, 0].value
        return u[0, 0].value
                
            
            
            
          
        