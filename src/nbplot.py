import multiprocessing as mp
import time

import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import numpy as np
import random
from process import ProcessPlotter


class NBPlot:


    def __init__(self):
        '''
        manager = mp.Manager()
        fake_lifo = manager.list()
        fake_lifo = [ [ [] for i in range(1) ] for i in range(plot.num_robots) ]
        '''
        self.plot_pipe, plotter_pipe = mp.Pipe()
        self.plotter = ProcessPlotter()
        self.plot_process = mp.Process(
            target=self.plotter, args=(plotter_pipe,), daemon=True)
        self.plot_process.start()


    def plot(self, id, finished=False):
        send = self.plot_pipe.send
        if finished:
            send(None)
        else:
            data = np.empty(4, dtype=int)
            data[0] = id
            for i in range(1, 4):
                data[i] = random.randrange(-3, 3, 1)
            print('DATA: ' + str(data))
            send(data)


if __name__ == '__main__':
    pl = NBPlot()
    while True:
        for id in range(3):
            pl.plot(int(id))
            time.sleep(0.1)
    #pl.plot(finished=True)