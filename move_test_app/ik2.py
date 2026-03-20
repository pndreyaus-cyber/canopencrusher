import numpy as np

DOF = 6

# DH parameters: a, alpha, d, theta0
DH = np.array([
    [0.0000, -0.9073,  0.0360,  3.14159],
    [-0.2340, -2.1865, -0.0306, -1.57080],
    [-0.1050,  1.7719, -0.3853,  0.00000],
    [0.0995,   1.6899,  0.0317,  1.57080],
    [-0.3993,  0.9388,  0.0137,  1.57080],
    [-0.0133,  0.0000, -0.0627, -1.57080]
])

JOINT_MIN = np.array([-2.967, -1.570, -2.356, -3.141, -1.919, -3.141])
JOINT_MAX = np.array([ 2.967, 0.785,  2.356,  3.141,  1.919,  3.141])


# -----------------------------
# DH transform
# -----------------------------
def dh_link(a, alpha, d, theta):

    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d],
        [0,   0,      0,     1]
    ])


# -----------------------------
# Forward kinematics
# -----------------------------
def forward(q):

    T = np.eye(4)

    for i in range(DOF):

        a, alpha, d, theta0 = DH[i]

        link = dh_link(a, alpha, d, theta0 + q[i])

        T = T @ link

    return T


# -----------------------------
# Numerical Jacobian
# -----------------------------
def jacobian(q):

    h = 1e-4
    J = np.zeros((6, DOF))

    T0 = forward(q)
    p0 = T0[:3, 3]

    for i in range(DOF):

        qh = q.copy()
        qh[i] += h

        Ti = forward(qh)
        pi = Ti[:3, 3]

        J[:3, i] = (pi - p0) / h

        R0 = T0[:3, :3]
        Ri = Ti[:3, :3]

        dR = (Ri - R0) / h
        W = R0.T @ dR

        J[3, i] = W[2,1]
        J[4, i] = W[0,2]
        J[5, i] = W[1,0]

    return J


# -----------------------------
# Inverse kinematics
# -----------------------------
def inverse_kinematics(target_pos, q_init=None):

    if q_init is None:
        q = np.zeros(6)
    else:
        q = q_init.copy()

    target = np.eye(4)
    target[:3,3] = target_pos

    MAX_ITER = 150

    for _ in range(MAX_ITER):

        T = forward(q)

        ep = target[:3,3] - T[:3,3]

        ep_norm = np.linalg.norm(ep)

        if ep_norm < 1e-3:
            return q

        J = jacobian(q)

        e = np.zeros(6)
        e[:3] = ep

        mu = 1e-3

        A = J @ J.T + mu*np.eye(6)

        v = np.linalg.solve(A, e)

        dq = J.T @ v

        lam = 0.1

        q = q + lam*dq

        q = np.clip(q, JOINT_MIN, JOINT_MAX)

    return None


# -----------------------------
# USER FUNCTION
# -----------------------------
def solve_robot(x, y, z):

    q = inverse_kinematics(np.array([x, y, z]))

    if q is None:
        print("IK did not converge")
        return None

    return q