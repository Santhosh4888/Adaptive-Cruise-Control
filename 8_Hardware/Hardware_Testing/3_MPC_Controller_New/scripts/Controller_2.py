#!/usr/bin/env python3

import numpy as np
import cvxpy as cp
import H_Common_Params as CP

class MPC:
    def __init__(self, Np=8, Nc=5):

        self.Np = Np
        self.Nc = Nc

        self.state_dim = 2
        self.control_dim = 1

        self.desired_speed = CP.ego_max_v
        self.a_curr = 0.0

        # --- TUNING (FOR 20 km/h VEHICLE) ---
        self.Q = np.diag([20.0, 10.0])     # [distance, velocity]
        self.R = np.array([[5.0]])         # control effort
        self.S = 50.0                      # jerk penalty (IMPORTANT)
        self.P = 5000.0                    # CBF relaxation

        self.alpha = 0.4                   # CBF softness

    def get_acceleration(self, ego_values, preceeding_values=None):

        ego_pos, ego_vel = ego_values

        if preceeding_values:
            obs_pos, obs_vel = preceeding_values
        else:
            obs_pos, obs_vel = None, None

        x = cp.Variable((2, self.Np + 1))
        u = cp.Variable((1, self.Np))
        delta = cp.Variable((1, self.Np))

        cost = 0
        constraints = []

        # Initial state
        x0 = np.array([ego_pos, ego_vel])
        constraints += [x[:, 0] == x0]

        cur_obs_pos = obs_pos

        for k in range(self.Np):


            # ADAPTIVE VELOCITY
            if obs_pos is not None:
                v_ref = cp.minimum(self.desired_speed, obs_vel + 0.2)
            else:
                v_ref = self.desired_speed

            # SAFE DISTANCE
            d_safe = CP.Dd + CP.Td * x[1, k]

            # ERROR VECTOR
            if obs_pos is not None:
                sep_error = (cur_obs_pos - d_safe) - x[0, k]
            else:
                sep_error = -x[0, k]

            vel_error = v_ref - x[1, k]

            error_vec = cp.vstack([sep_error, vel_error])

            # COST FUNCTION
            cost += cp.quad_form(error_vec, self.Q)

            if k < self.Nc:
                cost += cp.quad_form(u[:, k], self.R)

            # JERK PENALTY
            if k == 0:
                du = u[0, k] - self.a_curr
            else:
                du = u[0, k] - u[0, k-1]

            cost += self.S * cp.square(du)

            #  SOFT CBF PENALTY
            cost += self.P * cp.square(delta[0, k])

            # DYNAMICS
            constraints += [
                x[0, k+1] == x[0, k] + x[1, k]*CP.sample_time + 0.5*u[0, k]*CP.sample_time**2,
                x[1, k+1] == x[1, k] + u[0, k]*CP.sample_time
            ]

            # LIMITS            
            constraints += [
                CP.a_min <= u[0, k],
                u[0, k] <= CP.a_max,
                0 <= x[1, k],
                x[1, k] <= CP.ego_max_v
            ]

            # ACTUATOR CONSTRAINT
            constraints += [
                0 <= u[0, k] / CP.A + x[1, k],
                u[0, k] / CP.A + x[1, k] <= CP.ego_max_v
            ]
         
            # JERK CONSTRAINT         
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
      
            # CBF (SOFT, STABLE)          
            if obs_pos is not None:

                h = cur_obs_pos - CP.Dd - CP.Td*x[1, k] - x[0, k]

                constraints += [
                    obs_vel - CP.Td*u[0, k] - x[1, k] >= -self.alpha*h - delta[0, k],
                    delta[0, k] >= 0
                ]

                # Predict obstacle motion
                cur_obs_pos += obs_vel * CP.sample_time
 
            # CONTROL HORIZON FREEZE       
            if k >= self.Nc:
                constraints += [u[:, k] == u[:, k-1]]
 
        # SOLVE 
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve(solver=cp.OSQP, verbose=False, max_iter=50000)

        if u[0, 0].value is None:
            return 0.0

        self.a_curr = u[0, 0].value
        return self.a_curr