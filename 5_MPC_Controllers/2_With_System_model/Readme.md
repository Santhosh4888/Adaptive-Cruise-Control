# MPC Controller explanation in Latex format

\subsection{MPC Controller Design}

In this study, a discrete-time Model Predictive Control (MPC) framework is designed for adaptive cruise control (ACC) in low-speed electric vehicles. The vehicle is modeled using a linear point-mass approximation with position and velocity as system states and acceleration as the control input. The system is discretized with a sampling time of  T=0.1 seconds. The discrete state-space model is given by:

\begin{equation}
\label{deqn_ex3a}
{x}_{k+1} = A {x}_k + B u_k
\end{equation}

where the state and input are defined as:

\begin{equation}
x_k = \begin{bmatrix}
d_k \\
v_k
\end{bmatrix},
\quad
u_k = acceleration
\end{equation}

where:


\begin{itemize}
    \item \( d_k \) is the relative distance between the obstacle and ego vehicle
    \item \( v_k \) is the velocity of the ego vehicle
    \item \( u_k \) is the acceleration at time step k
\end{itemize}


The system matrices are:

\begin{equation}
A = 
\begin{bmatrix}
1 & T \\
0 & 1
\end{bmatrix},
\quad
B = 
\begin{bmatrix}
0.5 T^2 \\
T
\end{bmatrix}
\end{equation}

The MPC problem is solved over a prediction horizon of
N=6 steps and control horizon of N=4 steps. The objective function minimises the weighted sum of predicted position and velocity errors relative to a reference
trajectory, along with the control effort. The quadratic cost function is defined as:

%\textbf{Cost Function:}

\begin{equation}
    J = \sum_{k=0}^{N_p - 1}
    \left\|
        \bm{Q} \left( \bm{x}_{\text{ref}} - \bm{S} \bm{x}_k \right)
    \right\|^2 
    + \sum_{k=0}^{N_c - 1} 
    \left\| \bm{R} u_k \right\|^2 
    + \sum_{k=0}^{N_p - 1}
    \left\| \bm{P} \delta_k \right\|^2
\end{equation}

Compute optimal control inputs $u_0, u_1, \dots, u_{N_p-1}$ to minimize tracking error, control effort, and constraint violations.


Where:
\[
 {x}_{\text{ref}} = 
\begin{bmatrix}
0 \\
v_{\text{des}}
\end{bmatrix}, \quad
{S} = 
\begin{bmatrix}
0 & 0 \\
0 & 1
\end{bmatrix},
\]

\begin{itemize}
    \item $\delta_k$ = relaxation variable at time $k$, used to soften safety constraints (CBF).
    \item ${Q}$: Weight on state tracking error (e.g., speed error).
    \item ${R}$: Weight on control effort (e.g., acceleration).
    \item ${P}$: Large weight on relaxation variable $\delta$, penalizing safety violation (used in CBF constraints).
    \item $1 \leq k \leq N_c$: Indicates only the first $N_c$ control steps are penalized, to focus on near-future actions.
\end{itemize}

The reference position and velocity are dynamically updated based on the lead vehicle's state and a time-headway policy. To ensure comfort and safety, the optimization problem includes constraints on acceleration, jerk, and velocity as follows:

\begin{subequations}\label{eq:7}
\begin{align}
\text{Input Constraints:} \quad & a_{\min} \leq a_k \leq a_{\max} \label{eq:7A} \\
\text{Velocity Constraints:} \quad & 0 \leq v_k \leq v_{\max} \label{eq:7B} \\
\text{Jerk Constraints:} \quad & \frac{u_0 - a_{\text{curr}}}{T_s} \in [j_{\min}, j_{\max}] \label{eq:7C} \\
& \frac{u_k - u_{k-1}}{T_s} \in [j_{\min}, j_{\max}], \quad k > 0 \label{eq:7D}
\end{align}
\end{subequations}

\text{Control Barrier Function (obstacle):}
\begin{equation}
\qquad
  v_{\text{obs}} - T_d u_k - v_k
     \;\ge\;
     -\,\alpha\bigl(\hat{s}_k - D_d - T_d v_k - s_k\bigr) - \delta_k
\end{equation}

The Control Barrier Function (CBF) constraint plays a critical role in maintaining collision avoidance and enforcing safe vehicle separation \cite{Chinelato2023}. Its functionality can be described as follows:

\begin{itemize}
    \item When the ego vehicle operates within the safe region, i.e., its distance from the obstacle is greater than the desired separation, the CBF constraint ensures that the system remains within this region.
    \item In the event of external disturbances, modeling errors, or numerical inaccuracies that push the vehicle into the unsafe region (where the separation falls below the desired threshold), the CBF enforces corrective action, such that the vehicle attempts to reestablish safety, provided it is physically feasible.
    \item Furthermore, the formulation guarantees convergence of the ego vehicle toward the desired separation distance, thereby ensuring long-term safety and stability of the closed-loop system.
\end{itemize}