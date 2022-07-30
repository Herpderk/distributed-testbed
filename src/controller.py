import matplotlib.pyplot as plt
from scipy import linalg as la
from simple_pid import PID
import numpy as np
import time


class LQR_PID:


    def __init__(self):
        self.time_step = 0.1  # s
        self.wheel_radius = 64.5  # mm
        self.bodycenter_X = 46.5  # mm
        self.bodycenter_Y = 93.0  # mm
        self.V = 300.0 # mm/s
        self.A = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]])
        self.B = np.array([[self.time_step, 0, 0],
                           [0, self.time_step, 0],
                           [0, 0, self.time_step]])
        self.K = self.K_matrix()


    def angular_vels(self, v0, v1):
        deltaTheta = np.arccos(
            (np.dot(v0, v1)) / (np.sqrt(v0.dot(v0))*np.sqrt(v1.dot(v1))))
        vel_matrix = (1/self.time_step) * np.array([v0[0], v1[1], time_step])
        dynamics_matrix = np.array([[1, -1, -(self.bodycenter_X+self.bodycenter_Y)],
                                    [1,  1,  (self.bodycenter_X +
                                              self.bodycenter_Y)],
                                    [1,  1, -(self.bodycenter_X +
                                              self.bodycenter_Y)],
                                    [1, -1,  (self.bodycenter_X+self.bodycenter_Y)]])
        ang_vel_matrix = (1/self.wheel_radius) * \
            np.matmul(dynamics_matrix, vel_matrix)
        return ang_vel_matrix


    def K_matrix(self):
        A = self.A
        B = self.B

        Q = 1 * np.array([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]])
        R = 1 * np.array([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]])

        P = la.solve_discrete_are(self.A, self.B, Q, R)
        K = la.inv(R) @ np.transpose(self.B) @ P
        return K


    def lqr_steer(self, path_pos, curr_pos, curr_spd):
        print('ideal position: ' + str(path_pos))

        error = path_pos - curr_pos
        print('error: ' + str(error))
        
        U = self.K @ error
        U = curr_spd * (U/np.linalg.norm(U))
        print('control action: ' + str(U))

        next_error = (self.A @ error) - (self.B @ U)
        print('next error: ' + str(next_error))

        next_pos = path_pos - next_error
        print('next pos: ' + str(next_pos))
        print()

        return np.array([U, next_pos])


    def pid_speed(self, pid, nxt_angles, curr_spd):
        avg_turn = np.average(nxt_angles)
        ideal_spd = self.V*((1 - avg_turn/np.pi)**25)
        
        pid.setpoint = ideal_spd
        nxt_spd = pid(curr_spd)

        print('NEXT SPEED: ' + str(nxt_spd))
        return nxt_spd


    def path_tracking(self, path):
    	#initialize plot
        ax1, plt = self.init_plot(path)
        #initialize pid object
        pid = PID(1, 0, 0.4, setpoint=self.V)
        pid.output_limits = (0, 1.5*self.V)
        # declare the starting position and iterate through waypoints
        pos = np.array([0, 0, 0])
        horizon = 8

        for i in range (np.size(path, 0) - horizon):
            wp = path[i]
            spd = 0

            while True:
                error = wp - pos
                error_mag = np.sqrt(np.einsum('i,i', error, error))
            	
                if error_mag > 20:
                    self.plot_path(ax1, plt, pos)
                    
                    nxt_angles = np.array([
                        abs(path[i+1][2] - path[i][2])
            	        ])
                    for j in range(2, horizon+1):
                        nxt_angles = np.append(nxt_angles, 
                        	[abs(path[i+j][2] - path[i+j-1][2])], 
                        	axis=0)

                    spd = self.pid_speed(pid, nxt_angles, spd)
                    state = self.lqr_steer(wp, pos, spd)
                    pos = state[1]
                    vel = state[0]
                else:
                    break


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
    

    def plot_path(self, ax1, plt, pos):
        ax1.scatter(pos[0], pos[1], c='red', s=5)
        ax1.set_xlim(0, 1000)
        ax1.set_ylim(0, 1000)
        #ax1.annotate(str(pos[2]) + ' rads', (pos[0], pos[1]))
        plt.draw()
        plt.pause(0.001)


    def path(self, waypoints):
        new_col = np.zeros((waypoints.shape[0], 1))
        for i in range(new_col.shape[0] - 1):
            x_diff = waypoints[i+1, 0] - waypoints[i, 0]
            y_diff = waypoints[i+1, 1] - waypoints[i, 1]
            new_col[i + 1] = np.arctan2(y_diff, x_diff)
        path = np.append(waypoints, new_col, 1)
        return path


def sine_wave(size, num_points):
    waypoints = np.array([[0, 0]])
    for t in range(1, num_points):
    	waypoints = np.append(waypoints, [[t, (size/2)*np.sin((5*(2*np.pi*t/num_points) - np.pi/2))+(size/2)]], axis = 0)
    return waypoints


def circle(size, num_points):
    waypoints = np.array([[0, 0]])
    for t in range(1, num_points):
    	waypoints = np.append(waypoints, [[(size/2) + (size/2)*np.cos(2*np.pi*t/num_points), 
    	    	                               (size/2) + (size/2)*np.sin(2*np.pi*t/num_points)]], axis = 0)
    return waypoints


if __name__ == "__main__":
    controller = LQR_PID()

    size = 1000
    num_points = 1000
    waypoints = sine_wave(size, num_points)

    path = controller.path(waypoints)
    controller.path_tracking(path)
    