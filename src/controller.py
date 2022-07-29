import matplotlib.pyplot as plt
from scipy import linalg as la
import numpy as np
import time


class LQR_Controller:


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


    def control_action(self, path_pos, curr_pos):
        print('ideal position: ' + str(path_pos))

        error = path_pos - curr_pos
        print('error: ' + str(error))

        U = self.K @ error
        print('control action: ' + str(U))
        
        U = self.V*(U/np.linalg.norm(U))

        next_error = (self.A @ error) - (self.B @ U)
        print('next error: ' + str(next_error))

        next_pos = path_pos - next_error
        print('next pos: ' + str(next_pos))

        return np.array([U, next_pos])


    def pid_speed(self, nxt_pts, curr_spd):
        std_dev = np.std(nxt_pts, axis=0)
        dev_mag = np.sqrt(np.einsum('i,i', std_dev, std_dev))

        cost = dev_mag**10
        nxt_spd = cost*
        print(cost)


    def path_tracking(self, path):
        ax1, plt = self.init_plot(path)
        # declare the starting position and iterate through waypoints
        pos = np.array([0, 0, 0])
        
        for i in range (np.size(path, 0)):
            '''
            nextXs = np.array([
                path[i][0], path[i+1][0], path[i+2][0], path[i+3][0], path[i+4][0], path[i+5][0]
                ])
            nextYs = np.array([
                path[i][1], path[i+1][1], path[i+2][1], path[i+3][1], path[i+4][1], path[i+5][1]
                ])
            nextThetas = np.array([
                path[i][2], path[i+1][2], path[i+2][2], path[i+3][2], path[i+4][2], path[i+5][2]
                ])
            nextX = np.average(nextXs)
            nextY = np.average(nextYs)
            nextTheta = np.average(nextThetas)
            wp = np.array([
                nextX, nextY, nextTheta
                ])
            '''
            wp = path[i]
            error = wp - pos
            error_mag = np.sqrt(np.einsum('i,i', error, error))

            while abs(error_mag > 20):
                self.plot_path(ax1, plt, pos)
                pos = self.control_action(wp, pos)[1]
                error = wp - pos
                error_mag = np.sqrt(np.einsum('i,i', error, error))


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

'''
deltaTheta = np.arccos(
            (np.dot(v0, v1)) / (np.sqrt(v0.dot(v0))*np.sqrt(v1.dot(v1))))
            '''
if __name__ == "__main__":
    new_controller = LQR_Controller()
    wp1 = np.array([
    	[-1,1],
    	[0, 0],
    	[1,1]
    	])
    wp2 = np.array([
        [0, 0],
        [1, 1],
        [2, 2]
    	])
    
    new_controller.pid_speed(wp1, 300)
    new_controller.pid_speed(wp2, 300)
    '''
    waypoints = np.array([[0, 0]])
    size = 1000
    num_points = 1000
    for t in range(1, num_points):
    	waypoints = np.append(waypoints, [[t, (size/2)*np.sin(1*t*(np.pi/180.))+(size/2)]], axis = 0)

    path = new_controller.path(waypoints)
    new_controller.path_tracking(path)
    '''