import multiprocessing as mp
import time

import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import numpy as np

class ProcessPlotter:


    def __init__(self):
        self.x_coords = np.empty(3)
        self.y_coords = np.empty(3)
        self.z_coords = np.empty(3)

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


    def terminate(self):
        plt.close('all')


    def call_back(self):
        while self.pipe.poll():
            coords = self.pipe.recv()
            id = coords[0]
            self.x_coords[id] = coords[1]
            self.y_coords[id] = coords[2]
            self.z_coords[id] = coords[3]

            self.update_plot(id)
            
        return True


    def init_plot(self):
        self.fig, self.ax1 = plt.subplots(figsize=(8, 7))
        self.ax1.set_xlim(-self.x_dim, self.x_dim)
        self.ax1.set_ylim(-self.y_dim, self.y_dim)

        plt.show(block = False)
        plt.pause(0.1)
        self.bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        self.fig.canvas.blit(self.fig.bbox)


    def update_plot(self, id):
        self.fig.canvas.restore_region(self.bg)
        (pt,) = self.ax1.plot(self.x_coords[id], self.y_coords[id], self.tracker_colors[id], markersize=5, label="robot" + str(id))
        pt.set_data(self.x_coords[id], self.y_coords[id])

        self.ax1.set_xlim(-self.x_dim, self.x_dim)
        self.ax1.set_ylim(-self.y_dim, self.y_dim)
        self.ax1.draw_artist(pt)

        self.fig.canvas.blit(self.fig.bbox)
        self.fig.canvas.flush_events()


    def __call__(self, pipe):
        print('starting plotter...')
        self.pipe = pipe
        
        self.init_plot()

        timer = self.fig.canvas.new_timer(interval=0)
        timer.add_callback(self.call_back)
        timer.start()

        print('...done')
        plt.show()