import numpy as np

def f(qp, x, u, d=[0.]*3, g=9.81, K_p_w=0.05, Ts=0.001): # L_p_w=0.05

    """
    state:
        x = {x,y,z,q0,q1,q2,q3,xd,yd,zd,p ,q ,r ,w0,w1,w2,w3}
             0 1 2 3  4  5  6  7  8  9  10 11 12 13 14 15 16
    
    control:
        u = {w0d,w1d,w2d,w3d}
             0   1   2   3

    kwargs:
    
        disturbance (simulated wind):
            d = {velW,qW0,qW1}
                0    1   2

        gravity: 9.81

        rotors proportional control gain at Ts of 0.001s: K_p_w

    NOTES:
    
    I integrated the proportional control of the rotors directly into the 
    equations of motion to more accurately reflect the closed loop system
    we will be controlling with a second outer loop. This inner loop is akin
    to the ESC which will be onboard many quadcopters which directly controls
    the rotor speeds to be what is commanded.
    """

    # inner loop for rotor speed control - assumes no drag on rotors

    # we change the ratio of the gain dependent on the used time-step to
    # approximately achieve the same results regardless of timestep
    K_p_w = 0.001 / Ts * K_p_w

    # just the P gain * error for rotors
    tau = K_p_w * (u - x[13:17])

    # instantaneous thrusts and torques generated by the current w0...w3
    # x[13:17] = np.clip(x[13:17], qp["minWmotor"], qp["maxWmotor"]) # this clip shouldn't occur within the dynamics
    th = qp['kTh'] * x[13:17] ** 2
    to = qp['kTo'] * x[13:17] ** 2

    # state derivates (from sympy.mechanics derivation)
    xd = np.stack(
        [
            x[7], x[8], x[9], # xd, yd, zd
            - 0.5 * x[10] * x[4] - 0.5 * x[11] * x[5] - 0.5 * x[6] * x[12], # q0d
              0.5 * x[10] * x[3] - 0.5 * x[11] * x[6] + 0.5 * x[5] * x[12], # q1d
              0.5 * x[10] * x[6] + 0.5 * x[11] * x[3] - 0.5 * x[4] * x[12], # q2d
            - 0.5 * x[10] * x[5] + 0.5 * x[11] * x[4] + 0.5 * x[3] * x[12], # q3d
            - (
                qp["Cd"]
                * np.sign(d[0] * np.cos(d[1]) * np.cos(d[2]) - x[7])
                * (d[0] * np.cos(d[1]) * np.cos(d[2]) - x[7]) ** 2
                - 2 * (x[3] * x[5] + x[4] * x[6]) * (th[0] + th[1] + th[2] + th[3])
            )
            / qp["mB"], # xdd
            - (
                qp["Cd"]
                * np.sign(d[0] * np.sin(d[1]) * np.cos(d[2]) - x[8])
                * (d[0] * np.sin(d[1]) * np.cos(d[2]) - x[8]) ** 2
                + 2 * (x[3] * x[4] - x[5] * x[6]) * (th[0] + th[1] + th[2] + th[3])
            )
            / qp["mB"], # ydd
            - (
                -qp["Cd"] * np.sign(d[0] * np.sin(d[2]) + x[9]) * (d[0] * np.sin(d[2]) + x[9]) ** 2
                - (th[0] + th[1] + th[2] + th[3])
                * (x[3] ** 2 - x[4] ** 2 - x[5] ** 2 + x[6] ** 2)
                + g * qp["mB"]
            )
            / qp["mB"], # zdd (the - in front turns increased height to be positive - SWU)
            (
                (qp["IB"][1,1] - qp["IB"][2,2]) * x[11] * x[12]
                - qp["usePrecession"] * qp["IRzz"] * (x[13] - x[14] + x[15] - x[16]) * x[11]
                + (th[0] - th[1] - th[2] + th[3]) * qp["dym"]
            )
            / qp["IB"][0,0], # pd
            (
                (qp["IB"][2,2] - qp["IB"][0,0]) * x[10] * x[12]
                + qp["usePrecession"] * qp["IRzz"] * (x[13] - x[14] + x[15] - x[16]) * x[10]
                + (th[0] + th[1] - th[2] - th[3]) * qp["dxm"]
            )
            / qp["IB"][1,1], #qd
            ((qp["IB"][0,0] - qp["IB"][1,1]) * x[10] * x[11] - to[0] + to[1] - to[2] + to[3]) / qp["IB"][2,2], # rd
            tau[0] / qp["IRzz"], tau[1] / qp["IRzz"], tau[2] / qp["IRzz"], tau[3] / qp["IRzz"] # w0d ... w3d
        ]
    )

    return xd

if __name__ == "__main__":

    from params import quad_params as qp

    x = np.array([0,0,0,1,0,0,0,0,0,0,0,0,0,*[522.9847140714692]*4])
    u = np.array([0.01,-0.01,0.01,-0.01])
    xd = f(qp, x, u)

    print('fin')