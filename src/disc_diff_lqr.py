# BACKGROUND
# A controller that combines an LQR for steering with PID speed control for optimal path tracking.
# The LQR's control policy is found using SciPy's solver for the continuous-time algebraic Riccati equation.
# The PID is provided by the simple-pid python package, and the setpoint of the speed adjusts for turn angles. 
# This controller is intended for 2-wheel differential drive systems.
# Controller design source: https://digitallibrary.sdsu.edu/islandora/object/sdsu%3A24363

# INSTRUCTIONS
# Initialize LQR_PID object with the wheel radius, width, height, and desired maximum velocity.
# Path must be an array of (x,y) arrays and inputted into gen_path to be usable for the controller.
# Run path_tracking, an example can be found in main. 

import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from scipy import linalg as la
from simple_pid import PID
import numpy as np
import time
from live_plot import LivePlot


class LQR_PID:


    def __init__(self, radius, width, height, v, spd_ctrl):
        self.wheel_radius = radius # mm
        self.center_X = width/2    # mm
        self.center_Y = height/2   # mm
        self.max_V = v             # mm/s
        self.speed_control = spd_ctrl
        

    # The solution to the optimal control policy u = K*x
    def K_matrix(self, Wo):
        A = np.array([
            [0, 0, -np.cos(Wo*self.time_step)],
            [0, 0, -np.sin(Wo*self.time_step)],
            [0, 0, 0]
            ])
        B = np.array([
            [-np.sin(Wo*self.time_step), 0],
            [ np.cos(Wo*self.time_step), 0],
            [0, 1]
            ])
        # Q punishes inaccuracy
        Q = 1 * np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
            ])
        # R punishes actuation
        R = 1 * np.array([
            [1, 0],
            [0, 1]
            ])
        
        P = la.solve_continuous_are(A, B, Q, R)
        K = la.inv(R) @ B.T @ P
        return A, B, K


    def lqr_steer(self, path_pos, pos, Wo):
        error = path_pos - pos
        A, B, K = self.K_matrix(Wo)
        U = K @ error
        V = U[0]
        W = U[1]

        state_change = (A @ error) - (B @ U)
        #nxt_err = error + state_change*self.time_step
        #nxt_pos = path_pos - nxt_err
        nxt_pos = path_pos + state_change*self.time_step

        print('pos: ' + str(pos))
        print('error: ' + str(error))
        print('next position: ' + str(nxt_pos))
        print('vel: ' + str(V))
        print('angular vel: ' + str(W))
        print('state change: ' + str(state_change))
        print()
        return nxt_pos, W


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
        W = 0
        spd = 0
        vel = np.array([0, 0, 0])   
        horizon = 6

        for i in range (np.size(path, 0)):
            if np.size(path, 0) - i < horizon:
                horizon = np.size(path, 0) - i - 1
            wp = path[i]

            while True:
                error = wp - pos
                error_mag = np.sqrt(np.einsum('i,i', error, error))
                error_lim = 50 #10.75*self.R1[0][0] + 0.2275*self.max_V - 45
                # IMPORTANT: if error limit is too small, infinite oscillation occurs
                if error_mag > error_lim:
                    net_turn = path[i+horizon][2] - path[i][2]
                    #spd = self.pid_speed(self.pid, net_turn, spd)

                    pos, W = self.lqr_steer(wp, pos, W)
                    #ang_spds = self.motor_spds(vel)

                    plot.update(0, pos)
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
    controller = LQR_PID(radius=64.5, width=46.5, height=93.0, v=300.0, spd_ctrl=False)
    '''
    path = np.array([-5, -5, -5])
    pos = np.array([0, 0, 0])
    Wo = 0
    controller.lqr_steer(path, pos, Wo)
    '''
    size = 900
    num_points = 900
    waypoints = sine_wave(size, num_points)
    path = controller.gen_path(waypoints)
    controller.path_tracking(path)

