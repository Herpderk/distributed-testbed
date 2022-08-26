from multiprocess import Process
from multiprocess.managers import BaseManager
from queue import LifoQueue
from live_plot import LivePlot
import time
import numpy as np  


def clear_q(queue):
    while not queue.empty():
        try:
            queue.get(False)
        except Empty:
            continue
        queue.task_done()


def tracking(Qs):
    try:
        while True:
            for id in range(len(Qs)):
                rand_coord = 3*np.random.rand(3)
                Qs[id].put(rand_coord)

    except KeyboardInterrupt:
        print('interrupted!')


def plotting(Qs):
    new_plot = LivePlot()
    try:
        while True:
            #st = time.time()
            for id in range(len(Qs)):
                #print(lifo[id])
                if Qs[id].qsize() > 0:
                    st = time.time()
                    pos = Qs[id].get()
                    new_plot.update(id , pos)
                    clear_q(Qs[id])
                    et = time.time()
                    print(et - st)
                    #print('pos: ' + str(pos))

    except KeyboardInterrupt:
        print('interrupted!')


# create manager that knows how to create and manage LifoQueues
class MyManager(BaseManager):
    pass
MyManager.register('LifoQueue', LifoQueue)


if __name__ == "__main__":
    manager = MyManager()
    manager.start()

    plot = LivePlot()
    plot.close()

    Qs = ()
    n = plot.num_robots
    for id in range(n):
        locals()['lifo' + str(id)] = manager.LifoQueue(maxsize=10)
        Qs = Qs + (locals()['lifo' + str(id)],)
    print(Qs)
    
    # creating new processes
    track_process = Process(target=tracking, args=(Qs,))
    plot_process = Process(target=plotting, args=(Qs,))
  
    # running processes
    track_process.start()
    plot_process.start()
    
    # wait until processes finish
    track_process.join()
    plot_process.join()
    manager.shutdown()
    
   