# BACKGROUND
# A controller that implements  an LQR for optimal path tracking.
# The LQR's control policy is found using an iterative solution for the discrete-time algebraic Riccati equation.
# The dynamical system punishes change in linear and angular velocities to minimize high frequency inputs.
# This controller is intended for 2-wheel differential drive systems.

# INSTRUCTIONS
# Initialize DifferentialLQR object with the wheel radius.
# Waypoints must be an array of [x,y] arrays and inputted into gen_path to be usable for the controller.
# Run path_tracking, an example can be found in main. 

import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from scipy import linalg as la
import numpy as np
import time
from live_plot import LivePlot


class DifferentialLQR:


    def __init__(self, radius):
        self.wheel_radius = radius # mm
        self.time_step = 0.1       # s


    def solve_DARE(self, A, B, Q, R):
        X = Q
        maxiter = 150
        eps = 0.01

        for i in range(maxiter):
            Xn = A.T @ X @ A - A.T @ X @ B @ \
                la.inv(R + B.T @ X @ B) @ B.T @ X @ A + Q
            if (abs(Xn - X)).max() < eps:
                break
            X = Xn
        return Xn


    # The solution to the optimal control policy u = K*x
    def K_matrix(self, theta):
        A = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
            ])
        B = np.array([
            [np.cos(theta)*self.time_step, 0],
            [np.sin(theta)*self.time_step, 0],
            [0, self.time_step]
            ])
        # Q punishes inaccuracy
        Q = 1 * np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
            ])
        # R punishes actuation magnitude
        R = 10**-1000 * np.array([
            [1, 0],
            [0, 1]
            ])

        A1 = np.block([
            [A,                       B],
            [np.zeros((2,3)), np.eye(2)]
            ])
        B1 = np.block([
            [B        ],
            [np.eye(2)]
            ])
        Q1 = np.block([
            [Q,         np.zeros((3,2))],
            [np.zeros((2,3)),         R]
            ])
        #R1 punishes change in actuation
        R1 = np.array([
            [1, 0],
            [0, 0.7]
            ])
        
        P = self.solve_DARE(A1, B1, Q1, R1)
        K = la.inv(R1) @ B1.T @ P
        return A1, B1, K


    def lqr_steer(self, path_pos, pos, vel):
        error = path_pos - pos
        state = np.block([error, vel])
        theta = pos[2]

        A, B, K = self.K_matrix(theta)
        U = K @ state
        nxt_state = (A @ state) - (B @ U)
        nxt_state = np.split(nxt_state, [3, 5])

        nxt_err = nxt_state[0]
        nxt_vel = nxt_state[1]
        nxt_pos = path_pos - nxt_err

        if abs(nxt_vel[1]) >= np.pi:
            if nxt_vel[1] > 0:
                nxt_vel[1] = np.pi
            if nxt_vel[1] < 0:
                nxt_vel[1] = -np.pi

        print('vel: ' + str(-nxt_vel[0]))
        print('ROTATIONAL VEL: ' + str(-nxt_vel[1]))
        print('pos: ' + str(nxt_pos))
        print()
        return nxt_pos, nxt_vel


    def gen_path(self, waypoints):
        new_col = np.zeros((waypoints.shape[0], 1))
        for i in range(new_col.shape[0] - 1):
            x_diff = waypoints[i+1, 0] - waypoints[i, 0]
            y_diff = waypoints[i+1, 1] - waypoints[i, 1]
            angle = np.arctan2(y_diff, x_diff)
            new_col[i + 1] = (angle)

        path = np.append(waypoints, new_col, 1)
        return path


    def path_tracking(self, path):
        plot = LivePlot(path)
        pos = path[0] 
        vel = np.zeros(2)   

        for i in range (np.size(path, 0)):
            wp = path[i]
            error_lim = 50 

            while True:
                error = wp - pos
                error_mag = np.sqrt(np.einsum('i,i', error, error))
                # IMPORTANT: if error limit is too small, infinite oscillation occurs
                if error_mag > error_lim:
                    pos, vel = self.lqr_steer(wp, pos, vel)
                    #ang_spds = self.motor_spds(vel)
                    plot.update(0, pos)

                    if abs(vel[0]) < 10:
                        error_lim += error_lim
                    time.sleep(0.03)
                else:
                    break


# example path functions
def sine_wave(size, num_points):
    waypoints = np.empty((0, 2), float)
    for t in range(num_points):
        waypoints = np.append(waypoints, [[t, 
            (size/2)*np.sin((5*(2*np.pi*t/num_points) - np.pi/2))+(size/2)]], axis = 0)
    return waypoints


def circle(size, num_points):
    waypoints = np.empty((0, 2), float)
    for t in range(num_points):
        waypoints = np.append(waypoints, [[(size/2) + (size/2)*np.cos(2*np.pi*t/num_points), 
                                           (size/2) + (size/2)*np.sin(2*np.pi*t/num_points)]], axis = 0)
    return waypoints


def lissajous(size, num_points):
    waypoints = np.empty((0, 2), float)
    for t in range(num_points):
        waypoints = np.append(waypoints, [[(size/2) + (size/2)*np.cos(7*2*np.pi*t/num_points), 
                             (size/2) + (size/2)*np.sin(5*2*np.pi*t/num_points)]], axis = 0)
    return waypoints


if __name__ == '__main__':
    size = 900
    num_points = 1000
    waypoints = lissajous(size, num_points)

    controller = DifferentialLQR(radius=64.5)
    path = controller.gen_path(waypoints)
    controller.path_tracking(path)
