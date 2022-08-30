import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import numpy as np
import random
import time


class LivePlot:


    def __init__(self, path):
        self.x_lim = np.array([-10, 1000]) #mm
        self.y_lim = np.array([-10, 1000]) #mm
        self.z_lim = np.array([0, 1000]) #mm
        # colors and number of robots, index is the id
        self.tracker_colors = np.array([
            'rP'
        ])
        self.num_robots = len(self.tracker_colors)
        # labels for each robot
        self.labels = np.empty(self.num_robots, dtype=str)
        for i in range(self.num_robots):
            self.labels[i] = ('robot' + str(i))
        
        self.x_coords = np.zeros(self.num_robots)
        self.y_coords = np.zeros(self.num_robots)
        self.z_coords = np.zeros(self.num_robots)
        self.init_plot(path)


    def init_plot(self, path):
        self.fig, self.ax1 = plt.subplots(figsize=(8, 7))
        self.ax1.set_xlim(self.x_lim[0], self.x_lim[1])
        self.ax1.set_ylim(self.y_lim[0], self.y_lim[1])

        wp_X = np.empty((1), float)
        wp_Y = np.empty((1), float)
        for wp in path:
            wp_X = np.append(wp_X, wp[0])
            wp_Y = np.append(wp_Y, wp[1])
        self.ax1.scatter(wp_X, wp_Y, c='blue', s=30)

        plt.show(block = False)
        plt.pause(0.1)
        self.bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        self.fig.canvas.blit(self.fig.bbox)


    def update(self, id, position):
        # update coordinates
        self.x_coords[id] = position[0]
        self.y_coords[id] = position[1]
        #self.z_coords[id] = position[2]

        self.fig.canvas.restore_region(self.bg)
        # draw each marker
        for i in range(self.num_robots):
            (pt,) = self.ax1.plot(self.x_coords[i], self.y_coords[i], self.tracker_colors[i], markersize=5, label="robot" + str(i))
            pt.set_data(self.x_coords[i], self.y_coords[i])
            self.ax1.set_xlim(self.x_lim[0], self.x_lim[1])
            self.ax1.set_ylim(self.y_lim[0], self.y_lim[1])
            self.ax1.draw_artist(pt)    
        # animation stuff
        self.fig.canvas.blit(self.fig.bbox)
        self.fig.canvas.flush_events()


    def close(self):
        plt.close()


if __name__ == '__main__':
    new_plot = Live_Plot()
    while True:
        for i in range(3):
            new_plot.update(i, np.array([random.randrange(0, 2, 1),
                                        random.randrange(0, 2, 1),
                                        random.randrange(0, 2, 1)]))