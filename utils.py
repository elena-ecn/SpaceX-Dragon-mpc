import numpy as np


def desired_trajectory(x0, xg, N, dt):
    N_circle = 65
    N_pre = 10
    p_circle = create_circle(N_circle)
    p_pre_circle = np.linspace(x0[:3], p_circle[0], num=N_pre + 1)
    p_post1 = np.linspace(p_circle[-1], np.array([xg[0],-3,xg[2]]), num=9)
    p_post2 = np.linspace(np.array([xg[0],-3,xg[2]]), xg[:3], num=18)
    p_post_circle = np.vstack((p_post1, p_post2[1:, :]))
    positions = np.vstack((p_pre_circle, p_circle[1:-1, :], p_post_circle))
    velocities = np.diff(positions, axis=0)/dt
    velocities = np.vstack((velocities, np.zeros(3)))
    X_desired = np.array([np.hstack([positions[i], velocities[i]]) for i in range(N)])

    assert len(X_desired) == N
    return X_desired


def desired_trajectory_long(x0, xg, N, dt):
    if N % 2 == 0:  # N is even
        return np.vstack((desired_trajectory(x0, xg, int(N/2), dt), np.array([xg for i in range(int(N/2))])))
    else:
        raise TypeError("N needs to be even, N: {}".format(N))


def create_circle(N_circ):
    R = 1.1
    theta_vec = np.linspace(0-np.pi/2, 2*np.pi+np.pi/2, num=N_circ)
    return np.array([([R*np.cos(theta),-3,R*np.sin(theta)+3]) for theta in theta_vec])


def state_estimate(xi, xg):
    """Adds noise to the current state to mimic a state estimator."""
    if np.linalg.norm(xi-xg) < 1:
        return xi
    else:
        position_sigma = 0.01    # m
        velocity_sigma = 0.0001  # m/s
        return xi + np.hstack((position_sigma*np.random.randn(3), velocity_sigma*np.random.randn(3)))


def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[1]],
                     [-v[2], v[1], 0]])


def dcm_from_phi(phi):
    theta = np.linalg.norm(phi)
    if abs(theta)>1e-12:
        r = phi/theta
    else:
        r = np.zeros(3)

    Q = np.eye(3) + np.sin(theta)*skew(r) + (1.0 - np.cos(theta))*skew(r)@skew(r)
    return Q


def thruster_model(xi, xg, u):
    if np.linalg.norm(xi-xg) < 1:
        return u  # assume no thruster errors within 1 m
    else:
        misalignment = dcm_from_phi(np.deg2rad(3)*np.array([0.3,0.6,-0.8]))
        scale = np.diag([0.95,1.03,1.01])
        return misalignment*scale@u
