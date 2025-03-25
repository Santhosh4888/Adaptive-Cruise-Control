import numpy as np
import cvxpy as cp 
import Common_Params as CP  

class MPC:
    
    def __init__(self, Np = 6, Nc = 4):
        
        self.Np = Np                                         # Prediction horizon
        self.Nc = Nc                                         # Control horizon
        self.state_dim = 2                                   # State consists of seperation and velocity of ego vehicle
        self.contro_dim = 1                                  # Control just has the acceleration of the ego vehicle
        self.desired_speed = CP.ego_max_v                    # Obstacle velocity of the vehicle
        self.a_curr = 0.0                                    # Current acceleration of the vehicle
            
    def get_acceleration(self, ego_values, preceeding_values = None):
        
        ego_pos, ego_vel = ego_values
        if preceeding_values:
            obs_pos, obs_vel = preceeding_values
        else:
            obs_pos, obs_vel = None, None
        
        R = np.eye(1)                                                     # For defining a quadratic cost on the control action
        x = cp.Variable((self.state_dim, self.Np + 1))                    # Np + 1 for handling Np steps in prediction horizon and 1 for initial state
        u = cp.Variable((self.contro_dim, self.Np))
        delta = cp.Variable((self.contro_dim, self.Np), nonneg = True)
        
        alphav, alphah, psc = 0.1, 0.1, np.eye(1) * 1e-5
        
        if obs_pos:
            x0 = np.array([ego_pos, ego_vel])                             # Define the current state (initial state)
            cur_obs_pos = obs_pos
            cost = 0
            constraints = []
            for k in range(self.Np):
                x_ref = np.array([cur_obs_pos - CP.Dd, self.desired_speed])
                S = np.array([[1, CP.Td], [0, 1]])
                
                #cost += cp.quad_form(x_ref - S @ x[:, k], Q)              # Defining the cost function
                xerror = x_ref - S @ x[:, k]
                
                constraints += [2 * (xerror[1]) * u[0, k] <= - alphav * (xerror[1]) ** 2 + delta[0, k]] # CLF constraints
                constraints += [cp.SOC(delta[0, k], alphav * (xerror[1]) ** 2)]
                constraints += [- x[1, k] - CP.Td * u[0, k] >= - alphah * (xerror[0])]
            
                if k <= self.Nc: 
                    cost += cp.quad_form(u[:, k], R)
                cost += cp.quad_form(delta[:, k], psc)
                
                constraints += [x[0, k + 1] == x[0, k] + (x[1, k] * CP.sample_time + 0.5 * u[0, k] * CP.sample_time ** 2)]
                constraints += [x[1, k + 1] == x[1, k] + u[0, k] * CP.sample_time]
                
                if k >= self.Nc:                                          # This is done to make all the control actions after Control horizon same as control action at the time instant of control horizon
                    constraints += [u[:, k] == u[:, k - 1]]
                
                constraints += [CP.a_min <= u[0, k], u[0, k] <= CP.a_max] # Constraints on acceleration
                
                constraints += [0.0 <= x[1, k], x[1, k] <= CP.ego_max_v]  # Constraints on state of the vehicle
                constraints += [cur_obs_pos - 0.5 >= x[0, k]]             # Constraints on state of the vehicle
                
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
                
                #cost += cp.quad_form(x[:, k] - x_ref, Q)                  # Defining the cost function
                xerror = x_ref - x[:, k]
                
                constraints += [2 * (xerror[1]) * u[0, k] <= - alphav * (xerror[1]) ** 2 + delta[0, k]] # CLF constraints
                
                if k <= self.Nc: 
                    cost += cp.quad_form(u[:, k], R)
                cost += cp.quad_form(delta[:, k], psc)
                
                constraints += [x[0, k + 1] == 0]
                constraints += [x[1, k + 1] == x[1, k] + u[0, k] * CP.sample_time]
            
                if k >= self.Nc:
                    constraints += [u[:, k] == u[:, k - 1]]
                
                constraints += [CP.a_min <= u[0, k], u[0, k] <= CP.a_max] # Constraints on acceleration
                
                constraints += [0.0 <= x[1, k], x[1, k] <= CP.ego_max_v]  # Constraints on state of the vehicle

                if k == 0:
                    constraints += [CP.dec_Jerk_limit <= (u[0, k] - self.a_curr) / CP.sample_time, (u[0, k] - self.a_curr) / CP.sample_time <= CP.acc_Jerk_limit]
                else:
                    constraints += [CP.dec_Jerk_limit <= (u[0, k] - u[0, k - 1]) / CP.sample_time, (u[0, k] - u[0, k - 1]) / CP.sample_time <= CP.acc_Jerk_limit]
            
            constraints += [x[:, 0] == x0]
            constraints += [0.0 <= x[1, self.Np], x[1, self.Np] <= CP.ego_max_v]
        
        problem = cp.Problem(cp.Minimize(cost), constraints)             # Solving for the optimization problem
        problem.solve(solver = cp.GUROBI, verbose = False)
        self.a_curr = u[0, 0].value
        return u[0, 0].value
                
            
            
            
          
        