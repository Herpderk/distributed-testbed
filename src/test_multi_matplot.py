from multiprocess import Process 
from multiprocess import Manager
from live_plot import Live_Plot
import numpy as np
import time
import random


def tracking(lifo, plot):
    while True:
        for id in range(3):
            pos = np.random.rand(3)
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
                    update_func(plot, id, pos)
                    
def update_func(plot, id, pos):
    plot.update(id , pos)

        
  
if __name__ == '__main__':
    plot = Live_Plot()
    manager = Manager()
    fake_lifo = manager.list()
    fake_lifo = [ [ [] for i in range(1) ] for i in range(plot.num_robots) ]

    print('PSEUDO LIFO: ' + str(fake_lifo))
    for i in range(plot.num_robots):
        fake_lifo[i][0] = [0,0,0]

    # starting processes
    track_proc = Process(target=tracking, args=(fake_lifo, plot,))
    track_proc.start()
    update_proc = Process(target=updating, args=(fake_lifo, plot,))
    update_proc.start()
    # wait until processes finish
    track_proc.join()
    update_proc.join()
    plot_proc.join()