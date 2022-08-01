# BACKGROUND
# A controller that combines an LQR for steering with PID speed control for optimal path tracking.
# The LQR's control policy is found using SciPy's solver for the discrete-time algebraic Riccati equation.
# The PID is provided by the simple-pid python package, and the setpoint of the speed adjusts for sharp turns. 
# This controller is intended for 4-wheel mecanum drive systems.

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


class LQR_PID:


    def __init__(self, radius, width, height, v):
        self.wheel_radius = radius # mm
        self.center_X = width/2    # mm
        self.center_Y = height/2   # mm
        self.max_V = v                 # mm/s
        self.time_step = 0.1       # s
        self.K1 = self.K_matrix()


    # The solution to the optimal control policy u = K*x
    def K_matrix(self):
        A = np.eye(3)
        B = self.time_step*np.eye(3)
        Q = 1 * np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
            ])
        #punishes speed
        R = 0.1 * np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
            ])

        self.A1 = np.block([
            [A,                   B],
            [np.zeros((3,3)), np.eye(3)]
            ])
        self.B1 = np.block([
            [B        ],
            [np.eye(3)]
            ])
        Q1 = np.block([
            [Q,         np.zeros((3,3))],
            [np.zeros((3,3)),         R]
            ])
        #punishes turning
        self.R1 = 4 * np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
            ])

        P = la.solve_discrete_are(self.A1, self.B1, Q1, self.R1)
        K1 = la.inv(self.R1) @ self.B1.T @ P
        return K1


    # Continuously adjusts the setpoint of the pid based on turn angles in the horizon
    def pid_speed(self, pid, nxt_turns, curr_spd):
        costs = np.empty((1), float)
        for theta in nxt_turns:
            if np.cos(theta) >= 0:
                costs = np.append(costs, [abs(0.5*np.sin(theta))])
            elif np.cos(theta) < 0:
                costs = np.append(costs, [abs(0.5 - 0.5*np.cos(theta))]) 

        avg_cost = np.average(costs)
        #print('cost: ' + str(avg_cost))
        ideal_spd = self.max_V*((1 - avg_cost)**6)
        #print('setpoint: ' + str(ideal_spd))
        
        pid.setpoint = ideal_spd
        nxt_spd = pid(curr_spd)

        #print('next speed: ' + str(nxt_spd))
        return nxt_spd


    # Calculates u, inputs the pid speed, and calculates the next state A*x - B*u
    def lqr_steer(self, path_pos, pos, vel):
        error = path_pos - pos
        state = np.block([
            error, vel
        ])

        U = self.K1 @ state
        #U = curr_spd * (U/np.linalg.norm(U))
        nxt_state = (self.A1 @ state) - (self.B1 @ U)

        nxt_err, nxt_vel = np.split(nxt_state, 2)
        nxt_pos = path_pos - nxt_err
        nxt_vel = self.max_V * nxt_vel / np.sqrt(np.einsum('...i,...i', nxt_vel, nxt_vel))
        nxt_spd = np.sqrt(np.einsum('...i,...i', nxt_vel, nxt_vel))

        print('pos:' + str(nxt_pos))
        print('vel:' + str(nxt_vel))
        print('spd:' + str(nxt_spd))

        return nxt_pos, nxt_vel


    # Uses inverse kinematics to calculate the angular velocity of each wheel
    def motor_spds(self, vel):
        kinematics = np.array([
            [1, -1, -(self.center_X + self.center_Y)],
            [1,  1,  (self.center_X + self.center_Y)],
            [1,  1, -(self.center_X + self.center_Y)],
            [1, -1,  (self.center_X + self.center_Y)]
            ])

        ang_vel = (1/self.wheel_radius) * kinematics @ vel
        print('motor speeds: ' + str(ang_vel))
        print()
        return ang_vel


    # Adds angles between (x,y) waypoints to be used for steering
    def gen_path(self, waypoints):
        new_col = np.zeros((waypoints.shape[0], 1))

        for i in range(new_col.shape[0] - 1):
            x_diff = waypoints[i+1, 0] - waypoints[i, 0]
            y_diff = waypoints[i+1, 1] - waypoints[i, 1]

            angle = np.arctan2(y_diff, x_diff)
            new_col[i + 1] = (angle)
        
        path = np.append(waypoints, new_col, 1)
        return path

    
    def init_plot(self, path):
        fig, ax1 = plt.subplots(figsize=(8, 7))
        backend = matplotlib.get_backend()
        x = 50
        y = 50
        if backend == 'TkAgg':
            fig.canvas.manager.window.wm_geometry("+%d+%d" % (x, y))
        elif backend == 'WXAgg':
            fig.canvas.manager.window.SetPosition((x, y))
        else:
            fig.canvas.manager.window.move(x, y)

        wp_X = np.empty((1), float)
        wp_Y = np.empty((1), float)
        for wp in path:
            wp_X = np.append(wp_X, wp[0])
            wp_Y = np.append(wp_Y, wp[1])
        ax1.scatter(wp_X, wp_Y, c='blue', s=30)

        plt.show(block = False)
        plt.pause(0.1)
        bg = fig.canvas.copy_from_bbox(fig.bbox)
        fig.canvas.blit(fig.bbox)
        return ax1, fig, bg

   
    def plot_path(self, ax1, fig, bg, pos):
        (pt,) = ax1.plot(pos[0], pos[1], 'rP', markersize=5)
        pt.set_data(pos[0], pos[1])

        fig.canvas.restore_region(bg)
        ax1.draw_artist(pt)

        fig.canvas.blit(fig.bbox)
        fig.canvas.flush_events()


    # increase prediction horizon to decelerate earlier
    def path_tracking(self, path):
        ax1, fig, bg = self.init_plot(path)
        pid = PID(0.4, 0.0, 0.0, setpoint=self.max_V)
        pid.output_limits = (0, self.max_V)
        
        pos = path[0] 
        vel = np.array([0, 0, 0])   
        horizon = 10

        for i in range (np.size(path, 0) - horizon):
            if np.size(path, 0) - i < horizon:
        	    horizon = np.size(path, 0) - i

            wp = path[i]
            spd = 0

            while True:
                error = wp - pos
                error_mag = np.sqrt(np.einsum('i,i', error, error))
                error_lim = -42.5 + 10.75*self.R1[0][0] + 0.2275*self.max_V 
            	# IMPORTANT: if error limit is too small, infinite oscillation occurs
                if error_mag > error_lim:
                    nxt_turns = np.empty((1), float)
                    for j in range(horizon):
                        nxt_turns = np.append(nxt_turns, 
                        	[(path[i+j+1][2] - path[i+j][2])], axis=0)

                    spd = self.pid_speed(pid, nxt_turns, spd)
                    pos, vel = self.lqr_steer(wp, pos, vel)
                    ang_spds = self.motor_spds(vel)
                    self.plot_path(ax1, fig, bg, pos)
                    time.sleep(0.025)
                else:
                    break


# example path functions
def sine_wave(size, num_points):
    waypoints = np.empty((0, 2), float)
    for t in range(num_points):
    	waypoints = np.append(waypoints, [[t, 
    		(size/2)*np.sin((10*(2*np.pi*t/num_points) - np.pi/2))+(size/2)]], axis = 0)
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
        waypoints = np.append(waypoints, [[(size/2) + (size/2)*np.cos(5*2*np.pi*t/num_points), 
    	                     (size/2) + (size/2)*np.sin(2*2*np.pi*t/num_points)]], axis = 0)
    return waypoints
    

if __name__ == "__main__":
    # wheel radius, width, height, and max velocity in terms of mm and s
    controller = LQR_PID(64.5, 46.5, 93.0, 300.0)
    # make sure path has points in the form of (x,y) or (x,y,theta)
    
    size = 1000
    num_points = 1000
    waypoints = sine_wave(size, num_points)
    # if points are (x,y) then run it through gen_path
    path = controller.gen_path(waypoints)
    controller.path_tracking(path)
    