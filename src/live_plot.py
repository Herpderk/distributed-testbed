import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import numpy as np
import random
import time


class Live_Plot:

    def __init__(self):
        self.x_dim = 3 #mm
        self.y_dim = 3 #mm
        self.z_dim = 3 #mm
        # colors and number of robots, index is the id
        self.tracker_colors = np.array([
            'rP', 'gP', 'bP'
        ])
        self.num_robots = len(self.tracker_colors)
        # labels for each robot
        self.labels = np.empty(self.num_robots, dtype=str)
        for i in range(self.num_robots):
            self.labels[i] = ('robot' + str(i))
        
        self.x_coords = np.zeros(self.num_robots)
        self.y_coords = np.zeros(self.num_robots)
        self.z_coords = np.zeros(self.num_robots)
        '''
        plt.ion()
        fig = plt.figure(figsize=(12, 5))
        self.ax1 = fig.add_subplot(1, 2, 1, projection='3d')
        self.ax2 = fig.add_subplot(1, 2, 2)
        '''
        self.init_plot()

    def init_plot(self):
        self.fig, self.ax1 = plt.subplots(figsize=(8, 7))
        self.ax1.set_xlim(-self.x_dim, self.x_dim)
        self.ax1.set_ylim(-self.y_dim, self.y_dim)

        plt.show(block = False)
        plt.pause(0.1)
        self.bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        self.fig.canvas.blit(self.fig.bbox)


    def update(self, id, position):
        # update coordinates
        self.x_coords[id] = position[0]
        self.y_coords[id] = position[1]
        self.z_coords[id] = position[2]

        self.fig.canvas.restore_region(self.bg)
        # draw each marker
        for i in range(self.num_robots):
            (pt,) = self.ax1.plot(self.x_coords[i], self.y_coords[i], self.tracker_colors[i], markersize=5, label="robot" + str(i))
            pt.set_data(self.x_coords[i], self.y_coords[i])
            self.ax1.set_xlim(-self.x_dim, self.x_dim)
            self.ax1.set_ylim(-self.y_dim, self.y_dim)
            self.ax1.draw_artist(pt)    
        # animation stuff
        self.fig.canvas.blit(self.fig.bbox)
        self.fig.canvas.flush_events()
        '''
        # clear and redraw plot
        self.ax1.clear()
        self.ax1.scatter3D(self.x_coords, self.y_coords, self.z_coords, c=self.colors)
        # set axis limits
        self.ax1.set_xlim(0, self.x_dim)
        self.ax1.set_ylim(0, self.y_dim)
        self.ax1.set_zlim(0, self.z_dim)
        # annotate points
        for i in range(self.num_robots):
            self.ax1.text(
                self.x_coords[i], self.y_coords[i], self.z_coords[i], self.labels[i])

        self.ax2.clear()
        self.ax2.scatter(self.x_coords, self.y_coords, c=self.colors)

        self.ax2.set_xlim(0, self.x_dim)
        self.ax2.set_ylim(0, self.y_dim)
        for i, label in enumerate(self.labels):
            self.ax2.annotate(label, (self.x_coords[i], self.y_coords[i]))

        plt.draw()
        #plt.show()
        plt.pause(0.03)
        '''


if __name__ == '__main__':
    new_plot = Live_Plot()
    while True:
        for i in range(3):
            new_plot.update(i, np.array([random.randrange(0, 2, 1),
                                              random.randrange(0, 2, 1),
                                              random.randrange(0, 2, 1)]))