# BACKGROUND
# A controller that combines an LQR solution for steering with PID speed control for optimal path tracking.
# The LQR's solution is found using SciPy's solver for the discrete-time algebraic Riccati equation.
# The PID's solution is provided by the simple-pid python package, the setpoint of the speed adjusts for turns. 
# This controller is intended for 4-wheel mecanum drive systems, will work on solutions for differential drive.

# INSTRUCTIONS
# Initialize LQR_PID object with the wheel radius, width, height, and desired maximum velocity.
# Path must be an array of x,y arrays and inputted into gen_path to be usable for the controller.
# Run path_tracking, an example can be found in main. 

import matplotlib.pyplot as plt
from scipy import linalg as la
from simple_pid import PID
import numpy as np
import time


class LQR_PID:


    def __init__(self, radius, width, height, v):
        self.time_step = 0.1       # s
        self.wheel_radius = radius # mm
        self.center_X = width/2    # mm
        self.center_Y = height/2   # mm
        self.V = v                 # mm/s
        self.A = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]])
        self.B = np.array([[self.time_step, 0, 0],
                           [0, self.time_step, 0],
                           [0, 0, self.time_step]])
        self.K = self.K_matrix()

    # The solution to the optimal control policy u = K*x
    def K_matrix(self):
        A = self.A
        B = self.B

        Q = 1 * np.array([
        	[1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
            ])
        R = 1 * np.array([
        	[1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
            ])

        P = la.solve_discrete_are(self.A, self.B, Q, R)
        K = la.inv(R) @ np.transpose(self.B) @ P
        return K

    # Continuously adjusts the setpoint of the pid based on turn angles in the horizon
    def pid_speed(self, pid, nxt_angles, curr_spd):
        avg_turn = np.average(nxt_angles)
        #The exponent causes sharper deceleration, decrease if it's too aggressive
        ideal_spd = self.V*((1 - avg_turn/np.pi)**25)
        
        pid.setpoint = ideal_spd
        nxt_spd = pid(curr_spd)

        print('next speed: ' + str(nxt_spd))
        return nxt_spd

    # Calculates u, inputs the pid speed, and calculates the next state A*x - B*u
    def lqr_steer(self, path_pos, curr_pos, curr_spd):
        #print('ideal position: ' + str(path_pos))

        error = path_pos - curr_pos
        #print('error: ' + str(error))
        
        U = self.K @ error
        U = curr_spd * (U/np.linalg.norm(U))
        print('control action: ' + str(U))

        next_error = (self.A @ error) - (self.B @ U)
        #print('next error: ' + str(next_error))

        next_pos = path_pos - next_error
        #print('next pos: ' + str(next_pos))

        return np.array([U, next_pos])

    # Uses inverse kinematics to calculate the angular velocity of each wheel
    def motor_spds(self, vel):
        kinematics = np.array([
        	[1, -1, -(self.center_X + self.center_Y)],
            [1,  1,  (self.center_X + self.center_Y)],
            [1,  1, -(self.center_X + self.center_Y)],
            [1, -1,  (self.center_X + self.center_Y)]
            ])
        ang_vel = (1/self.wheel_radius) * kinematics @ vel
        print('angular speeds: ' + str(ang_vel))
        print()
        return ang_vel

    # Adds angles between (x,y) waypoints to be used for steering
    def gen_path(self, waypoints):
        new_col = np.zeros((waypoints.shape[0], 1))
        for i in range(new_col.shape[0] - 1):
            x_diff = waypoints[i+1, 0] - waypoints[i, 0]
            y_diff = waypoints[i+1, 1] - waypoints[i, 1]
            new_col[i + 1] = np.arctan2(y_diff, x_diff)
        path = np.append(waypoints, new_col, 1)
        return path

    # Initializes the plot
    def init_plot(self, path):
    	# initialize plot
        plt.ion()
        fig = plt.figure(figsize=(6, 6))
        ax1 = fig.add_subplot(1, 1, 1)
        # plot waypoints from path
        wp_X = []
        wp_Y = []
        for wp in path:
            wp_X.append(wp[0])
            wp_Y.append(wp[1])
        ax1.scatter(wp_X, wp_Y, c='blue', s=25)
        return ax1, plt

    # Plots the position at every given time frame
    def plot_path(self, ax1, plt, pos):
        ax1.scatter(pos[0], pos[1], c='red', s=5)
        ax1.set_xlim(0, 1000)
        ax1.set_ylim(0, 1000)
        #ax1.annotate(str(pos[2]) + ' rads', (pos[0], pos[1]))
        plt.draw()
        plt.pause(0.001)

    # Where everything comes together
    def path_tracking(self, path):
    	#initialize plot and pid
        ax1, plt = self.init_plot(path)
        pid = PID(1, 0, 0.4, setpoint=self.V)
        pid.output_limits = (0, 1.5*self.V)
        # declare the starting position, prediction horizon, and iterate through waypoints
        pos = path[0]
        horizon = 8

        for i in range (np.size(path, 0) - horizon):
            wp = path[i]
            spd = 0

            while True:
                error = wp - pos
                error_mag = np.sqrt(np.einsum('i,i', error, error))
            	# IMPORTANT: if error magnitude is too small, infinite oscillation occurs
                if error_mag > 20:
                    nxt_angles = np.array([
                        abs(path[i+1][2] - path[i][2])
            	        ])
                    for j in range(2, horizon+1):
                        nxt_angles = np.append(nxt_angles, 
                        	[abs(path[i+j][2] - path[i+j-1][2])], 
                        	axis=0)

                    spd = self.pid_speed(pid, nxt_angles, spd)
                    state = self.lqr_steer(wp, pos, spd)
                    vel, pos = state[0], state[1]
                    ang_spds = self.motor_spds(vel)
                    self.plot_path(ax1, plt, pos)
                else:
                    break

# example path functions
def sine_wave(size, num_points):
    waypoints = np.array([[0, 0]])
    for t in range(1, num_points):
    	waypoints = np.append(waypoints, [[t, 
    		(size/2)*np.sin((5*(2*np.pi*t/num_points) - np.pi/2))+(size/2)]], axis = 0)
    return waypoints


def circle(size, num_points):
    waypoints = np.array([[(size/2) + (size/2)*np.cos(0), 
    	    	                      (size/2) + (size/2)*np.sin(0)]])
    for t in range(1, num_points):
    	waypoints = np.append(waypoints, [[(size/2) + (size/2)*np.cos(2*np.pi*t/num_points), 
    	    	                           (size/2) + (size/2)*np.sin(2*np.pi*t/num_points)]], axis = 0)
    return waypoints


if __name__ == "__main__":
    # wheel radius, width, height, and max velocity in terms of mm and s
    controller = LQR_PID(64.5, 46.5, 93.0, 300.0)

    size = 900
    num_points = 1000
    waypoints = circle(size, num_points)

    path = controller.gen_path(waypoints)
    controller.path_tracking(path)
    