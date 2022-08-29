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
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from scipy import linalg as la
from simple_pid import PID
import numpy as np
import time
#import control


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
        self.Q = 1 * np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 0.01]
            ])
        #R punishes actuation
        self.R = 1 * np.array([
            [0.000000001, 0],
            [0, 1]
            ])

        self.A1 = np.block([
            [self.A,                       self.B],
            [np.zeros((2,3)), np.eye(2)]
            ])
        self.B1 = np.block([
            [self.B        ],
            [np.eye(2)]
            ])
        self.Q1 = np.block([
            [self.Q,         np.zeros((3,2))],
            [np.zeros((2,3)),         self.R]
            ])
        #R1 punishes turning
        self.R1 = 1 * np.array([
            [1, 0],
            [0, 1]
            ])
        '''
        P = la.solve_discrete_are(self.A1, self.B1, self.Q1, self.R1)
        #P = control.dare(self.A1, self.B1, self.Q1, self.R1)[0]
        print(np.shape(self.B1.T))
        print('P: ' + str(P))
        K1 = la.inv(R1) @ self.B1.T @ P
        '''
        P, N = self.solve_DARE(self.A, self.B, self.Q, self.R)
        K = [None] * N
        for i in range(N):
            K[i] = -np.linalg.pinv(self.R + self.B.T @ P[i+1] @ self.B) @ self.B.T @ P[i+1] @ self.A

        K1 = K[N-1]
        
        return K1


    # Continuously adjusts the setpoint of the pid based on turn angles in the horizon
    def pid_speed(self, pid, net_turn, curr_spd):
        if self.speed_control:
            if np.cos(net_turn) >= 0:
                    cost = abs(0.5*np.sin(net_turn))
            elif np.cos(net_turn) < 0:
                    cost = abs(0.5 - 0.5*np.cos(net_turn)) 

            ideal_spd = 90*self.max_V*((1 - cost)**8)
            pid.setpoint = ideal_spd
            
            if (curr_spd > 0.2*self.max_V) & (ideal_spd > 0.2*90*self.max_V):
                pid.reset()
                pid.tunings = (0.01, 0.0, 0.00)
            elif (curr_spd < 0.2*self.max_V) & (ideal_spd > 0.2*90*self.max_V):
                pid.reset()
                pid.tunings = (0.01, 0.005, 0.0)

            nxt_spd = pid(curr_spd, self.time_step)
            return nxt_spd
        else:
            nxt_spd = self.max_V
            return nxt_spd


    # Calculates u, inputs the pid speed, and calculates the next state A*x - B*u
    def lqr_steer(self, path_pos, pos, vel, spd):
        #vel[0] = spd
        print('path : ' + str(path_pos))
        print('curr angle: ' + str(pos[2]))

        error = path_pos - pos
        theta = error[2]
        '''
        state = np.block([error, vel])
        theta = state[2]
        '''
        K = self.K_matrix(theta)
        U =  -K @ error
        #U[0] = vel[0] - spd
        print('U: ' + str(U))
       
        nxt_state = (self.A @ error) - (self.B @ U)
        nxt_state = np.split(nxt_state, [3,5])
        
        nxt_pos = path_pos - nxt_state
        nxt_vel = np.array([0,0])
        '''
        nxt_err = nxt_state[0]
        nxt_vel = nxt_state[1]
        print('spd and angular vel: ' + str(nxt_vel))
        #nxt_vel[0] = spd
        
        nxt_pos = path_pos - nxt_err
        #print('spd:' + str(nxt_spd))
        #print('vel: ' + str(nxt_vel))
        print('pos: ' + str(nxt_pos))
        print()
        #print('path: ' + str(path_pos))
        '''
        return nxt_pos, nxt_vel


    # Uses differential drive kinematics to calculate the angular velocity of each wheel
    def motor_spds(self, vel):
        return 0

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
        x, y = 900, 50
        if backend == 'TkAgg':
            fig.canvas.manager.window.wm_geometry("+%d+%d" % (x, y))
        elif backend == 'WXAgg':
            fig.canvas.manager.window.SetPosition((x, y))
        else:
            fig.canvas.manager.window.move(x, y)
        plt.xlim(0, 1100)

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
        plt.xlim(0, 1100)

        fig.canvas.restore_region(bg)
        ax1.draw_artist(pt)
        fig.canvas.blit(fig.bbox)
        fig.canvas.flush_events()


    # increase prediction horizon to decelerate earlier
    def path_tracking(self, path):
        ax1, fig, bg = self.init_plot(path)
        pid = PID(0.04, 0.0, 0.0, setpoint = 90*self.max_V)
        pid.output_limits = (0, self.max_V)
        
        pos = path[0] 
        vel = np.array([0, 0])   
        horizon = 7

        for i in range (np.size(path, 0) - horizon):
            if np.size(path, 0) - i < horizon:
        	    horizon = np.size(path, 0) - i

            wp = path[i]
            spd = 0

            while True:
                error = wp - pos
                
                xy_error = np.array([
                    error[0], error[1]
                    ])
                error_mag = np.sqrt(np.einsum('i,i', xy_error, xy_error))
                error_lim = 30 #10.75*self.R1[0][0] + 0.2275*self.max_V - 45
            	# IMPORTANT: if error limit is too small, infinite oscillation occurs
                if error_mag > error_lim:
                    net_turn = path[i+horizon][2] - path[i][2]
                    spd = self.pid_speed(pid, net_turn, spd)
                    pos, vel = self.lqr_steer(wp, pos, vel, spd)

                    self.plot_path(ax1, fig, bg, pos)
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
    

if __name__ == "__main__":
    # wheel radius, width, height, and max velocity in terms of mm and s
    # to disable speed control (constant speed), enter False
    controller = LQR_PID(radius=64.5, width=46.5, height=93.0, v=300.0, spd_ctrl=False)
    # make sure path has points in the form of (x,y) or (x,y,theta)
    size = 1000
    num_points = 1000
    waypoints = sine_wave(size, num_points)
    # if points are (x,y) then run it through gen_path
    path = controller.gen_path(waypoints)
    controller.path_tracking(path)
    