from matplotlib.pyplot import figure
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

import numpy as np
import time
import random


class Plot_3D:


    def __init__ (self):
        # dimensions of 3d space in cm
        self.x_length_cm = 100
        self.y_length_cm = 100
        self.z_length_cm = 100
        # number of robots to track
        self.num_robots = 3
        self.x_coords = np.zeros(self.num_robots)
        self.y_coords = np.zeros(self.num_robots)
        self.z_coords = np.zeros(self.num_robots)
        # colors to identify each robot
        tracker_colors = {
            0:'red',
            1:'green',
            2:'blue'
        }
        self.colors = []
        for i in range(self.num_robots):
            self.colors.append(tracker_colors[i])
        # labels for each robot
        self.labels = []
        for i in range(self.num_robots):
            self.labels.append('robot' + str(i))
        #initialize figures
        plt.ion()
        fig = plt.figure(figsize=(12, 5))
        self.ax1 = fig.add_subplot(1, 2, 1, projection='3d')
        self.ax2 = fig.add_subplot(1, 2, 2)
        

    def update_plot(self, id, position):
        self.x_coords[id] = position[0]
        self.y_coords[id] = position[1]
        self.z_coords[id] = position[2]

        self.ax1.clear()
        self.ax1.scatter3D(self.x_coords, self.y_coords, self.z_coords, c=self.colors)

        self.ax1.set_xlim(0, self.x_length_cm)
        self.ax1.set_ylim(0, self.y_length_cm)
        self.ax1.set_zlim(0, self.z_length_cm)
        for i in range(self.num_robots):
            self.ax1.text(self.x_coords[i], self.y_coords[i], self.z_coords[i], self.labels[i])

        self.ax2.clear()
        self.ax2.scatter(self.x_coords, self.y_coords, c=self.colors)

        self.ax2.set_xlim(0, self.x_length_cm)
        self.ax2.set_ylim(0, self.y_length_cm)
        for i, label in enumerate(self.labels):
            self.ax2.annotate(label, (self.x_coords[i], self.y_coords[i]))

        plt.draw()
        plt.pause(0.1)


if __name__ == '__main__':
    new_plot = Plot_3D()
    print(new_plot.labels)
    while True:
        for i in range(3):
            new_plot.update_plot(i, np.array([random.randrange(0, 100, 1),
                                              random.randrange(0, 100, 1),
                                              random.randrange(0, 100, 1)]))