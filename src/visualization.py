from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import time
import random

class Plot_3D:


    def __init__ (self):
        # dimensions of 3d space in cm
        self.x_length_cm = 100
        self.y_length_cm = 100
        self.z_length_cm = 100

        # number of robots to track
        num_robots = 3
        # initialize coordinate arrays
        self.x_coords = np.zeros(num_robots)
        self.y_coords = np.zeros(num_robots)
        self.z_coords = np.zeros(num_robots)

        #initialize axes
        plt.ion()
        self.ax = plt.axes(projection='3d')


    def update_plot(self, id, position):
        self.x_coords[id] = position[0]
        self.y_coords[id] = position[1]
        self.z_coords[id] = position[2]

        self.ax.clear()
        self.ax.scatter3D(self.x_coords, self.y_coords, self.z_coords, c='red')
        self.ax.autoscale(False)
        self.ax.set_xlim(0, self.x_length_cm)
        self.ax.set_ylim(0, self.y_length_cm)
        self.ax.set_zlim(0, self.z_length_cm)
        plt.draw()
        plt.pause(0.1)


if __name__ == '__main__':
    new_plot = Plot_3D()
    while True:
        for i in range(3):
            new_plot.update_plot(i, np.array([random.randrange(0, 100, 1),
                                              random.randrange(0, 100, 1),
                                              random.randrange(0, 100, 1)]))