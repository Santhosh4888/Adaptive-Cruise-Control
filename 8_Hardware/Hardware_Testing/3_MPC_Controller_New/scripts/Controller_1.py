#!/usr/bin/env python3  
# A tracking-based MPC with hard safety constraints, which can lead to aggressive braking and infeasibility under tight conditions.
# This controller is designed to prioritize safety by enforcing hard constraints on the distance to the preceding vehicle. 
# However, this can lead to aggressive braking and infeasibility in scenarios where the preceding vehicle decelerates rapidly or is very close. 
# The controller may struggle to find a feasible solution that satisfies all constraints, 
# especially when the desired speed is high and the safety distance is not maintained.

# Need to Test to check its behaviour in realtime.

import numpy as np
import cvxpy as cp 
import H_Common_Params as CP 

class MPC:
    
    def __init__(self, Np=20, Nc=10):   # Increased horizon
        
        self.Np = Np
        self.Nc = Nc
        self.state_dim = 2
        self.control_dim = 1
        self.desired_speed = CP.ego_max_v
        self.a_curr = 0.0
            
    def get_acceleration(self, ego_values, preceeding_values=None):
        
        ego_pos, ego_vel = ego_values
        
        if preceeding_values:
            obs_pos, obs_vel = preceeding_values
        else:
            obs_pos, obs_vel = None, None
        
        w_d = 50
        w_v = 1
        R = 0.1
        P = 1e6
        
        x = cp.Variable((2, self.Np + 1))
        u = cp.Variable((1, self.Np))
        delta = cp.Variable((1, self.Np))
        
        x0 = np.array([ego_pos, ego_vel])
        
        cost = 0
        constraints = []
        
        cur_obs_pos = obs_pos if obs_pos is not None else None
        
        for k in range(self.Np):
            # --- COST ---
            if cur_obs_pos is not None:
                d = cur_obs_pos - x[0, k]
                d_safe = CP.Dd + CP.Td * x[1, k]
                
                cost += w_d * cp.square(d - d_safe)   # distance tracking
            
            cost += w_v * cp.square(self.desired_speed - x[1, k])
            
            if k < self.Nc:
                cost += R * cp.square(u[0, k])
            cost += P * cp.square(delta[0, k])
            
            # --- DYNAMICS ---
            constraints += [
                x[0, k+1] == x[0, k] + x[1, k]*CP.sample_time + 0.5*u[0, k]*CP.sample_time**2,
                x[1, k+1] == x[1, k] + u[0, k]*CP.sample_time
            ]
            
            # --- LIMITS ---
            constraints += [
                CP.a_min <= u[0, k],u[0, k] <= CP.a_max,0 <= x[1, k],x[1, k] <= CP.ego_max_v
            ]
            
            # Constraints on the acceleration based on the system model
            #constraints += [0.0 <= u[0, k] / CP.A + x[1, k], u[0, k] / CP.A + x[1, k] <= CP.ego_max_v] 
            
            # --- JERK ---
            if k == 0:
                constraints += [
                    CP.dec_Jerk_limit <= (u[0, k] - self.a_curr)/CP.sample_time,
                    (u[0, k] - self.a_curr)/CP.sample_time <= CP.acc_Jerk_limit
                ]
            else:
                constraints += [
                    CP.dec_Jerk_limit <= (u[0, k] - u[0, k-1])/CP.sample_time,
                    (u[0, k] - u[0, k-1])/CP.sample_time <= CP.acc_Jerk_limit
                ]
            
            # --- CBF ---
            if cur_obs_pos is not None:
                h = cur_obs_pos - CP.Dd - CP.Td*x[1, k] - x[0, k]
                
                constraints += [
                    obs_vel - CP.Td*u[0, k] - x[1, k] >= -1.0*h - delta[0, k],
                    delta[0, k] >= 0
                ]
                
                cur_obs_pos += obs_vel * CP.sample_time
            
            # Control horizon freeze
            if k >= self.Nc:
                constraints += [u[:, k] == u[:, k-1]]
        
        constraints += [x[:, 0] == x0]
        
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve(solver=cp.OSQP, verbose=False)
        
        if u[0, 0].value is None:
            return 0.0
        
        self.a_curr = u[0, 0].value
        return u[0, 0].value