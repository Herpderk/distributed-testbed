from multiprocess import Process 
from multiprocess import Manager
#from multiprocessing import Process
import NatNetClient
import plot
import coord_pub
import time
import random

def tracking(lifo):
    while True:
        for id in range(3):
            position = [random.randrange(0, 2, 1), random.randrange(0, 2, 1), random.randrange(0, 2, 1)]
            lifo[id].append(position)


def plotting(lifo, new_plot):
    while True:
        #st = time.time()
        if len(lifo) > 0:
            for id in range(len(lifo)):
                arr = lifo.pop()
                del lifo[id]
                print('ARRAY AFTER DELETION: ' + str(lifo[id]))
                pos = arr[0]

                if id == 0:
                    print('PLOTTING: ' + str(pos))
                new_plot.update(id , pos)
        
  

if __name__ == '__main__':
    manager = Manager()
    fake_lifo = manager.list()

    new_plot = plot.Plot_3D()
    n = new_plot.num_robots
    fake_lifo = manager.list()

    fake_lifo = [ [ [] for i in range(1) ] for i in range(n) ]
    print('PSEUDO LIFO: ' + str(fake_lifo))
    for i in range(n):
        fake_lifo[i][0] = [0,0,0]
    print(fake_lifo)

    
    # creating new processes
    track_process = Process(target=tracking, args=[fake_lifo])
    plot_process = Process(target=plotting, args=[fake_lifo, new_plot])
  
    # running processes
    track_process.start()
    plot_process.start()
    
    # wait until processes finish
    track_process.join()
    plot_process.join()
    