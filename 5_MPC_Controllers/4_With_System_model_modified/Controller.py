import numpy as np
import cvxpy as cp
import sys
import os
internal_WSM_dir = os.path.dirname(os.path.abspath(__file__))
internal_MPC_dir = os.path.abspath(os.path.join(internal_WSM_dir, '..'))
root_dir = os.path.abspath(os.path.join(internal_MPC_dir, '..'))
internal_CP_dir = os.path.abspath(os.path.join(root_dir, '6_Common_Paramaters'))
sys.path.append(internal_CP_dir)
import Common_Params as CP

class MPC:
    def __init__(self, Np=6, Nc=4):
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

        # Weight matrices
        q_sep = 10.0
        q_vel = 20.0
        r_ctrl = 30.0
        p_relax = 10000

        Q = np.diag([q_sep, q_vel])
        R = np.eye(1) * r_ctrl
        P = np.eye(1) * p_relax
        x = cp.Variable((self.state_dim, self.Np + 1))
        u = cp.Variable((self.control_dim, self.Np))
        delta = cp.Variable((self.control_dim, self.Np))

        if obs_pos is not None:
            x0 = np.array([ego_pos, ego_vel])
            cur_obs_pos = obs_pos
            cost = 0
            constraints = []
            for k in range(self.Np):
                # Desired separation (time gap + minimum distance)
                desired_sep = CP.Dd + CP.Td * x[1, k]

                # State error: [ (obs_pos - desired_sep) - ego_pos , desired_speed - ego_vel ]
                # Both terms are CVXPY expressions – combine them into a vector
                err1 = (cur_obs_pos - desired_sep) - x[0, k]          # separation error

                #err2 = self.desired_speed - x[1, k]                   # velocity error
                v_ref = cp.minimum(self.desired_speed, obs_vel + 0.1)
                
                err2 = v_ref - x[1,k]
                error_vec = cp.vstack([err1, err2])

                cost += cp.quad_form(error_vec, Q)

                if k < self.Nc:
                    cost += cp.quad_form(u[:, k], R)
                cost += cp.quad_form(delta[:, k], P)

                # Dynamics
                constraints += [x[0, k+1] == x[0, k] + x[1, k] * CP.sample_time +
                                0.5 * u[0, k] * CP.sample_time**2]
                constraints += [x[1, k+1] == x[1, k] + u[0, k] * CP.sample_time]

                if k >= self.Nc:
                    constraints += [u[:, k] == u[:, k-1]]

                constraints += [CP.a_min <= u[0, k], u[0, k] <= CP.a_max]
                constraints += [0.0 <= u[0, k] / CP.A + x[1, k], u[0, k] / CP.A + x[1, k] <= CP.ego_max_v] # Constraints on the acceleration based on the system model
                constraints += [0.0 <= x[1, k], x[1, k] <= CP.ego_max_v]

                # CBF constraint
                alpha = 0.2
                h = cur_obs_pos - CP.Dd - CP.Td * x[1, k] - x[0, k]
                constraints += [obs_vel - CP.Td * u[0, k] - x[1, k] >= -alpha * h - delta[0, k]]
                constraints += [delta[0, k] >= 0]

                # Jerk limits
                if k == 0:
                    constraints += [CP.dec_Jerk_limit <= (u[0, k] - self.a_curr) / CP.sample_time,
                                    (u[0, k] - self.a_curr) / CP.sample_time <= CP.acc_Jerk_limit]
                else:
                    constraints += [CP.dec_Jerk_limit <= (u[0, k] - u[0, k-1]) / CP.sample_time,
                                    (u[0, k] - u[0, k-1]) / CP.sample_time <= CP.acc_Jerk_limit]

                cur_obs_pos += obs_vel * CP.sample_time

            constraints += [x[:, 0] == x0]
            #removed the constarint on the final state to allow the optimizer to choose the best final state
            # constraints += [0.0 <= x[1, self.Np], x[1, self.Np] <= CP.ego_max_v]
            # constraints += [cur_obs_pos - 0.5 >= x[0, self.Np]]

        else:   # No obstacle
            x0 = np.array([0, ego_vel])
            cost = 0
            constraints = []
            for k in range(self.Np):
                # State error: [ 0 - ego_pos , desired_speed - ego_vel ]
                # But ego_pos is constrained to 0 by dynamics, so we only care about velocity.
                # However, to keep the same structure, we include position error.
                err1 = -x[0, k]        # 0 - x[0,k]
                err2 = self.desired_speed - x[1, k]
                error_vec = cp.vstack([err1, err2])

                cost += cp.quad_form(error_vec, Q)

                if k < self.Nc:
                    cost += cp.quad_form(u[:, k], R)

                # Dynamics (position is forced to zero)
                constraints += [x[0, k+1] == 0]
                constraints += [x[1, k+1] == x[1, k] + u[0, k] * CP.sample_time]

                if k >= self.Nc:
                    constraints += [u[:, k] == u[:, k-1]]

                constraints += [CP.a_min <= u[0, k], u[0, k] <= CP.a_max]
                constraints += [0.0 <= u[0, k] / CP.A + x[1, k], u[0, k] / CP.A + x[1, k] <= CP.ego_max_v] # Constraints on the acceleration based on the system model
                constraints += [0.0 <= x[1, k], x[1, k] <= CP.ego_max_v]

                # Jerk limits
                if k == 0:
                    constraints += [CP.dec_Jerk_limit <= (u[0, k] - self.a_curr) / CP.sample_time,
                                    (u[0, k] - self.a_curr) / CP.sample_time <= CP.acc_Jerk_limit]
                else:
                    constraints += [CP.dec_Jerk_limit <= (u[0, k] - u[0, k-1]) / CP.sample_time,
                                    (u[0, k] - u[0, k-1]) / CP.sample_time <= CP.acc_Jerk_limit]

            constraints += [x[:, 0] == x0]

        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve(solver=cp.OSQP, verbose=False, max_iter=50000)

        self.a_curr = u[0, 0].value if u[0, 0].value is not None else 0.0
        return self.a_curr