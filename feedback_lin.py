import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

import geometry
from params import quad_params as qp
from dynamics import f
from plotting import Animator

def vee_map(S):
    """
    Convert a skew-symmetric matrix to a vector.
    """
    return np.array([S[2,1], S[0,2], S[1,0]])

# Parameters
m = qp['mB']  # Mass of the quadrotor
g = 9.81       # Gravity acceleration
J = qp['IB']  # Inertia matrix of the quadrotor
B0 = qp['B0']      # Actuation matrix
B0_pinv = np.linalg.pinv(B0)  # Pseudo-inverse of B0

# Controller gains
Lambda = np.diag([1.0, 1.0, 1.0])  # Tuning parameter for position error
K = np.diag([2.0, 2.0, 2.0])       # Gain matrix for s
K_R = 1.0 * np.eye(3)              # Attitude control gain
K_omega = 0.1 * np.eye(3)          # Angular velocity control gain

# Initialize state and reference
xk = np.array([0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, *[522.9847140714692]*4])
rk = np.copy(xk)

# Simulation parameters - Ts = 0.03 is maximum for stability here
Ti, Tf, Ts = 0.0, 30.0, 0.03
t = np.arange(Ti, Tf, Ts)
x = [np.copy(xk)]
r = [np.copy(rk)]

# Initialize integral of position error
tilde_p_integral = np.zeros(3)

for tk in tqdm(t):

    # Desired Trajectory:
    # -------------------

    # cicular
    p_d = np.array([np.sin(tk), np.cos(tk), 0])
    dp_d = np.array([np.cos(tk), -np.sin(tk), 0])
    ddp_d = np.array([-np.sin(tk), -np.cos(tk), 0])

    # up and down
    # hz = 1
    # p_d = np.array([0,0,-np.sin(tk * hz)])
    # dp_d = np.array([0,0,-np.cos(tk * hz)])
    # ddp_d = np.array([0,0,np.sin(tk * hz)])

    # stand still
    # p_d = np.array([0,0,0])
    # dp_d = np.array([0,0,0])
    # ddp_d = np.array([0,0,0])

    # unpacking
    # ---------

    rk[0:3] = p_d  # Desired position

    # Extract current state
    p = xk[0:3]         # Position
    q = xk[3:7]         # Quaternion
    v = xk[7:10]        # Velocity
    omega = xk[10:13]   # Angular velocity

    # Calculate desired f_d via feedback lin (https://ieeexplore.ieee.org/document/9196800/)
    # --------------------------------------

    # Compute position tracking error and its integral
    tilde_p = p - p_d
    tilde_p_integral += tilde_p * Ts

    # Compute composite variable s
    s = v + 2 * Lambda @ tilde_p + Lambda @ Lambda @ tilde_p_integral

    # Compute reference velocity and its derivative
    v_r = dp_d - 2 * Lambda @ tilde_p - Lambda @ Lambda @ tilde_p_integral
    dv_r = ddp_d - 2 * Lambda @ (v - dp_d) - Lambda @ Lambda @ tilde_p

    # Estimate interaction forces (assuming zero for single vehicle)
    hat_f_a = np.zeros(3)

    # Compute desired total force
    f_d = m * dv_r - K @ s - m * np.array([0, 0, -g]) - hat_f_a

    # Calculate desired rotation matrix to achieve f_d
    # ------------------------------------------------

    # because we use this desired force to calculate the desired angle - if it is close to zero we have a 
    # badly defined problem. For that reason we have some control flow here to handle that edge case
    if np.linalg.norm(f_d) < qp["minThr"]:

        print('min thrust regime engaged')
        # Set desired orientation to whatever its currently at
        R_d = geometry.quaternion_to_rotation_matrix(q)
        T_d = qp["minThr"]

    else:

        # Compute desired thrust magnitude
        T_d = np.linalg.norm(f_d)

        # Normalize desired force to get desired body z-axis
        b3_d = f_d / T_d

        # Desired yaw angle (set to zero for simplicity)
        psi_d = 0.0
        a_yaw = np.array([np.cos(psi_d), np.sin(psi_d), 0])

        # Compute desired body x and y axes
        b2_d = np.cross(b3_d, a_yaw)
        b2_d /= np.linalg.norm(b2_d)
        b1_d = np.cross(b2_d, b3_d)

        # Assemble desired rotation matrix
        R_d = np.column_stack((b1_d, b2_d, b3_d))

    # Get attitude error - calculate desired torques - get final control
    # ------------------------------------------------------------------

    # Convert current quaternion to rotation matrix
    R = geometry.quaternion_to_rotation_matrix(q)

    # Compute attitude error
    e_R_matrix = 0.5 * (R_d.T @ R - R.T @ R_d)
    e_R = vee_map(e_R_matrix)

    # Compute angular velocity error (assuming desired angular velocity is zero)
    e_omega = omega

    # Compute desired torques
    tau_d = -K_R @ e_R - K_omega @ e_omega

    # Assemble desired wrench
    eta_d = np.concatenate(([T_d], tau_d))

    # Compute control input (squared motor speeds)
    uk = B0_pinv @ eta_d

    # Square root this for compatibility with my sim (also ensures sqrt of negative never happens)
    uk = np.clip(uk, qp["minWmotor"]**2, qp["maxWmotor"]**2)
    uk = np.sqrt(uk)

    # Step the system
    # ---------------

    xk += f(qp, xk, uk, Ts=Ts) * Ts

    # Clip the rotor speeds within limits
    xk[13:17] = np.clip(xk[13:17], qp["x_lb"][13:17], qp["x_ub"][13:17])

    # save
    # ----

    x.append(np.copy(xk))
    r.append(np.copy(rk))

x = np.stack(x)
r = np.stack(r)

animator = Animator(qp, x, t, r)
animator.animate()

plt.close()
plt.plot(x[:,0], x[:,1], label="xy")
plt.plot(x[:,0], x[:,2], label="xz")
plt.legend()
plt.savefig('test.png', dpi=500)
