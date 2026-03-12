import numpy as np
import cvxpy as cp
import os
import sys
internal_WSM_dir = os.path.dirname(os.path.abspath(__file__))
internal_MPC_dir = os.path.abspath(os.path.join(internal_WSM_dir, '..'))
root_dir = os.path.abspath(os.path.join(internal_MPC_dir, '..'))
internal_CP_dir = os.path.abspath(os.path.join(root_dir, '6_Common_Paramaters'))
sys.path.append(internal_CP_dir)
import Common_Params as CP


class MPC:

    def __init__(self, Np=6, Nc=4):
        self.Np = Np                                  # Prediction horizon
        self.Nc = Nc                                  # Control horizon (kept for compatibility)
        self.state_dim = 2
        self.control_dim = 1
        self.desired_speed = CP.ego_max_v
        self.a_curr = 0.0

        # Discrete-time system
        self.T = CP.sample_time
        self.A = np.array([[1, self.T],
                           [0, 1]])
        self.B = np.array([[0.5 * self.T ** 2],
                           [self.T]])

        # Lifted matrices
        self.A_bar, self.B_bar = self._build_prediction_matrices()

        # Velocity selector
        self.C_vel = np.kron(np.eye(self.Np), [0, 1]).reshape(self.Np, 2 * self.Np)

    def _build_prediction_matrices(self):
        A_bar = np.zeros((2 * self.Np, 2))
        B_bar = np.zeros((2 * self.Np, self.Np))

        for i in range(1, self.Np + 1):
            A_bar[2 * (i - 1):2 * i, :] = np.linalg.matrix_power(self.A, i)
            for j in range(i):
                B_bar[2 * (i - 1):2 * i, j] = (
                    np.linalg.matrix_power(self.A, i - j - 1) @ self.B
                ).flatten()
        return A_bar, B_bar

    def get_acceleration(self, ego_values, preceeding_values=None):

        ego_pos, ego_vel = ego_values

        if preceeding_values is not None:
            obs_pos, obs_vel = preceeding_values
        else:
            obs_pos, obs_vel = None, None

        # -------- Mode logic (ACC vs Cruise) --------
        if obs_pos is not None:
            d_rel = obs_pos - ego_pos
        else:
            d_rel = np.inf

        if d_rel < CP.Dd + CP.Td * ego_vel:
            # ACC mode
            d_safe = CP.Dd + CP.Td * ego_vel
            p_ref = obs_pos - d_safe
            v_ref = obs_vel
        else:
            # Cruise mode
            p_ref = ego_pos + self.desired_speed * self.T * self.Np
            v_ref = self.desired_speed

        X_ref = np.tile(np.array([p_ref, v_ref]), self.Np)

        # -------- Optimization variables --------
        U = cp.Variable(self.Np)

        x0 = np.array([ego_pos, ego_vel])
        X_pred = self.A_bar @ x0 + self.B_bar @ U

        # -------- Cost function --------
        w_p = 150.0
        w_v = 80.0
        w_u = 80.0

        Q = np.diag(np.concatenate([
            w_p * np.ones(self.Np),
            w_v * np.ones(self.Np)
        ]))

        cost = cp.quad_form(X_pred - X_ref, Q)
        cost += w_u * cp.sum_squares(U)

        # -------- Constraints --------
        constraints = []

        vel_pred = self.C_vel @ X_pred
        constraints += [vel_pred >= 0.0,
                        vel_pred <= CP.ego_max_v]

        constraints += [U >= CP.a_min,
                        U <= CP.a_max]

        # Jerk constraints
        for k in range(self.Np - 1):
            constraints += [
                (U[k + 1] - U[k]) / self.T <= CP.acc_Jerk_limit,
                (U[k] - U[k + 1]) / self.T <= -CP.dec_Jerk_limit
            ]

        # Initial jerk constraint
        constraints += [
            (U[0] - self.a_curr) / self.T <= CP.acc_Jerk_limit,
            (self.a_curr - U[0]) / self.T <= -CP.dec_Jerk_limit
        ]

        # -------- Solve --------
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)

        if U.value is None:
            a_cmd = 0.0
        else:
            a_cmd = float(U.value[0])

        self.a_curr = a_cmd
        return a_cmd
