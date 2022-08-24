from multiprocess import Process 
from multiprocess import Manager
import plotly_gui
import numpy as np
import time
import random


def tracking(lifo, plot):
    while True:
        for id in range(3):
            pos = np.random.rand(plot.num_robots)
            pos_list = pos.tolist()
            lifo[id].append(pos_list)


def updating(lifo, plot):
    while True:
        if len(lifo) > 0:
            print('LIFO: ' + str(lifo))
            for coords in lifo:
                for id in range(plot.num_robots):
                    pos = lifo[id].pop()
                    print('ARRAY BEFORE DELETION: ' + str(lifo[id]))
                    del lifo[id][:]
                    print('ARRAY AFTER DELETION: ' + str(lifo[id]))

                    if id == 0:
                        print('PLOTTING: ' + str(pos))
                    plot.update_coords(id , pos)


def plotting(app):
    app.run_server(port = 8080, debug = True)
        
  
if __name__ == '__main__':
    manager = Manager()
    fake_lifo = manager.list()
    fake_lifo = [ [ [] for i in range(1) ] for i in range(plotly_gui.plot.num_robots) ]

    print('PSEUDO LIFO: ' + str(fake_lifo))
    for i in range(plotly_gui.plot.num_robots):
        fake_lifo[i][0] = [0,0,0]

    # starting processes
    track_proc = Process(target=tracking, args=[fake_lifo, plotly_gui.plot])
    track_proc.start()
    update_proc = Process(target=updating, args=[fake_lifo, plotly_gui.plot])
    update_proc.start()
    plot_proc = Process(target=plotting, args=[plotly_gui.app])
    plot_proc.start()
  
    # wait until processes finish
    track_proc.join()
    update_proc.join()
    plot_proc.join()
    