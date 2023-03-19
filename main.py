import numpy as np
from scipy import linalg
import cvxpy as cp

import utils
from plotter import plot_trajectories


def continuous_dynamics():
    """Computes the continuous-time dynamics of the Dragon vehicle.

    State:    x = [r_x, r_y, r_z, v_x, v_y, v_z]
    Controls: u = [t_x, t_y, t_z]
    where r is the relative position of the Dragon spacecraft with respect to the ISS,
          v is the relative velocity, and t is the thrust on the spacecraft.
    Returns:
      - A(np.ndarray): The state matrix [6x6]
      - B(np.ndarray): The input matrix [6x3]
    """
    # Parameters
    mu = 3.986004418e14   # Standard gravitational parameter
    a = 6971100.0         # Semi-major axis of ISS
    n = np.sqrt(mu/a**3)  # Mean motion

    # Continuous time dynamics ẋ = Ax + Bu
    A = np.array([[0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 1],
                  [3*n**2, 0, 0, 0, 2*n, 0],
                  [0, 0, 0, -2*n, 0, 0],
                  [0, 0, -n**2, 0, 0, 0]])
    B = np.vstack((np.zeros((3,3)), 0.1*np.eye(3)))
    return A, B


def discretize_dynamics(A, B, dt):
    """Computes the discrete-time dynamics of the Dragon vehicle.

    x_{k+1} = Ad*x_k + Bd*u_k
    Since the continuous-time system is linear, we'll use the matrix exponential
    to discretize the dynamics.
    Inputs:
      - A(np.ndarray): The continuous-time state matrix [6x6]
      - B(np.ndarray): The continuous-time input matrix [6x3]
      - dt(float):     The discretization step
    Returns:
      - Ad(np.ndarray): The discrete-time state matrix [6x6]
      - Bd(np.ndarray): The discrete-time input matrix [6x3]
    """
    nx, nu = B.shape
    M1 = np.hstack((A, B))
    M2 = np.zeros((nu, nx+nu))
    M =  linalg.expm(np.vstack((M1, M2))*dt)  # Matrix exponential

    # Extract matrices from matrix exponential
    Ad = M[:nx, :nx]
    Bd = M[:nx, nx:]

    return Ad, Bd


def convex_mpc(A, B, X_ref_window, xic, x_goal, u_min, u_max, N_mpc):
    """Solves OCP as convex optimization problem for a time-horizon N_mpc.

    Inputs:
      - A(np.ndarray):      The discrete-time state matrix [nxn]
      - B(np.ndarray):      The discrete-time input matrix [nxm]
      - X_ref_window(np.ndarray):  The reference trajectory for the current window [nxN]
      - xic(np.ndarray):    The current 'initial' state (n,)
      - x_goal(np.ndarray): The goal state (n,)
      - u_min(np.ndarray):  The min input bound (m,)
      - u_max(np.ndarray):  The max input bound (m,)
      - N_mpc(int):         The MPC time horizon
    Returns:
      - (np.ndarray): The first control input to be applied to the robot (m,)
    """
    nx, nu = B.shape  # State and controls size

    # Cost function weights
    Q = np.eye(nx)
    R = np.eye(nu)

    # Decision variables
    X = cp.Variable((nx, N_mpc))
    U = cp.Variable((nu, N_mpc-1))

    # Objective function (quadratic)
    objective = 0
    for i in range(N_mpc-1):
        objective += (1/2)*cp.quad_form(X[:, i]-X_ref_window[:, i], Q) + (1/2)*cp.quad_form(U[:, i], R)

    objective += (1/2)*cp.quad_form(X[:, N_mpc-1]-X_ref_window[:, N_mpc-1], Q)

    # Constraints
    constraints = [X[:, 0] == xic]                           # Initial conditions
    constraints += [X[:, -1] == x_goal]                      # Final condition
    for i in range(N_mpc-1):
        constraints += [u_min <= U[:, i]]                    # Control bounds
        constraints += [U[:, i] <= u_max]                    # Control bounds
        constraints += [X[:, i+1] == A@X[:, i] + B@U[:, i]]  # Dynamics

    for i in range(N_mpc):
        constraints += [X[1, i] <= x_goal[1]]                # State constraints

    prob = cp.Problem(cp.Minimize(objective), constraints)
    prob.solve(solver='ECOS', verbose=False)
    return U.value[:, 0]


def simulation_MPC(x0, x_goal, X_ref, Ad, Bd, N, N_mpc, u_min, u_max):
    """Simulation with MPC controller."""

    N_sim = N + N_mpc   # Simulation timesteps
    nx, nu = Bd.shape   # State and controls size
    X_sim = np.zeros((nx, N_sim))
    X_sim[:, 0] = x0
    U_sim = np.zeros((nu, N_sim-1))

    for i in range(N_sim-1):
        # Get state estimate
        xi_estimate = utils.state_estimate(X_sim[:, i], x_goal)

        # Given a window of N_mpc timesteps, get current reference trajectory
        X_ref_tilde = X_ref[:, i:(i+N_mpc)]

        # Call convex mpc controller with state estimate
        u_mpc = convex_mpc(Ad, Bd, X_ref_tilde, xi_estimate, x_goal, u_min, u_max, N_mpc)

        # Commanded control goes into thruster model where it gets modified
        U_sim[:, i] = utils.thruster_model(X_sim[:, i], x_goal, u_mpc)

        # Simulate one step
        X_sim[:, i+1] = Ad@X_sim[:, i] + Bd@U_sim[:, i]

    return X_sim, U_sim


if __name__ == "__main__":
    Ts = 1.0                                                      # Discretization step
    N_mpc = 20                                                    # MPC window size
    N = 100                                                       # Simulation timesteps
    x0 = np.array([-2.0, -4.0, 2.0, 0, 0, 0])                     # Initial state
    x_goal = np.array([0, -0.68, 3.05, 0, 0, 0])                  # Goal state
    u_max = 0.4*np.ones(3)                                        # Upper bound for control inputs
    u_min = -u_max                                                # Lower bound for control inputs
    X_ref = utils.desired_trajectory_long(x0, x_goal, 2*N, Ts).T  # Reference trajectory to track (nx x 2N)

    # Discretize the dynamics
    A, B = continuous_dynamics()
    Ad, Bd = discretize_dynamics(A, B, Ts)

    # Simulate with MPC control
    X_sim, U_sim = simulation_MPC(x0, x_goal, X_ref, Ad, Bd, N, N_mpc, u_min, u_max)

    # Plot state & control trajectories
    plot_trajectories(X_sim, U_sim)
