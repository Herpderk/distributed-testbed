# BACKGROUND
# A controller that combines an LQR for steering with PID speed control for optimal path tracking.
# The LQR's control policy is found using SciPy's solver for the discrete-time algebraic Riccati equation.
# The PID is provided by the simple-pid python package, and the setpoint of the speed adjusts for turn angles. 
# This controller is intended for 4-wheel mecanu drive systems.

# INSTRUCTIONS
# Initialize LQR_PID object with the wheel radius, width, height, and desired maximum velocity.
# Path must be an array of (x,y) arrays and inputted into gen_path to be usable for the controller.
# Run path_tracking, an example can be found in main. 

import matplotlib
import matplotlib.pyplot as plt
from scipy import linalg as la
from simple_pid import PID
import numpy as np
import time
import control


class LQR_PID:


    def __init__(self, radius, width, height, v, spd_ctrl):
        self.wheel_radius = radius # mm
        self.center_X = width/2    # mm
        self.center_Y = height/2   # mm
        self.max_V = v             # mm/s
        self.speed_control = spd_ctrl
        self.time_step = 0.1       # s


    def solve_DARE(self, A, B, Q, R):
        N = 100
        P = [None] * (N + 1)
        Qf = Q
        P[N] = Qf
        for i in range(N, 0, -1):
            P[i-1] = Q + A.T @ P[i] @ A - (A.T @ P[i] @ B) @ np.linalg.pinv(
                R + B.T @ P[i] @ B) @ (B.T @ P[i] @ A)      
        return P, N


    # The solution to the optimal control policy u = K*x
    def K_matrix(self, theta):
        self.A = np.eye(3)
        self.B = self.time_step * np.array([
                [np.cos(theta), 0],
                [np.sin(theta), 0],
                [0,             1]
            ])
        Q = 1 * np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
            ])
        #R punishes actuation
        R = 1 * np.array([
            [1, 0],
            [0, 1]
            ])

        self.A1 = np.block([
            [self.A,             self.B],
            [np.zeros((2,3)), np.eye(2)]
            ])
        self.B1 = np.block([
            [self.B   ],
            [np.eye(2)]
            ])
        self.Q1 = np.block([
            [Q,         np.zeros((3,2))],
            [np.zeros((2,3)),         R]
            ])
        #R1 punishes turning
        self.R1 = 1 * np.array([
            [1, 0],
            [0, 1]
            ])
        '''
        #P = la.solve_discrete_are(A, B, Q, R)
        P = control.dare(self.A1, self.B1, Q1, R1)[0]
        print(np.shape(self.B1.T))
        print('P: ' + str(P))
        K1 = la.inv(R1) @ self.B1.T @ P
        '''
        P, N = self.solve_DARE(self.A, self.B, Q, R)
        K = [None] * N
        for i in range(N):
            K[i] = -np.linalg.pinv(R + self.B.T @ P[i+1] @ self.B) @ self.B.T @ P[i+1] @ self.A

        K1 = K[N-1]
        return K1


    def lqr_steer(self, path_pos, pos, vel, spd):
        error = path_pos - pos
        print('pos: ' + str(pos))
        print('error: ' + str(error))
        state = np.block([error, vel])
        theta = state[2]

        K = self.K_matrix(theta)
        U =  -K @ error
        print('U: ' + str(U))
        '''
        U = spd * U / np.sqrt(np.einsum('...i,...i', U, U))
        print('control action: ' + str(U))
        #print('error: ' + str(error))
        nxt_err = self.A @ error - self.B @ U
        nxt_pos = path_pos - nxt_err
        nxt_vel = U
        '''
        nxt_err = (self.A @ error) - (self.B @ U)
        '''
        nxt_state = np.split(nxt_state, [3,5])
        
        nxt_err = nxt_state[0]
        print('nxt error: ' + str(nxt_err))
        nxt_vel = nxt_state[1]
        nxt_vel[0] = spd
        '''
        nxt_pos = path_pos - nxt_err
        #print('spd:' + str(nxt_spd))
        #print('vel: ' + str(nxt_vel))
        print('pos: ' + str(nxt_pos))
        
        return nxt_pos#, nxt_vel


if __name__ == '__main__':
    controller = LQR_PID(radius=64.5, width=46.5, height=93.0, v=300.0, spd_ctrl=False)
    path_pos = np.array([
        100, 100 , 1.5
        ])
    pos = np.array([
        0.075, 1.055, 0.015
        ])
    vel = np.array([
        300, 0.0
        ])
    spd = 300
    controller.lqr_steer(path_pos, pos, vel, spd)