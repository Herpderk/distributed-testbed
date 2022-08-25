from multiprocess import Process, Manager
from multiprocess.managers import BaseManager
from NatNetClient import NatNetClient
from queue import LifoQueue
from live_plot import LivePlot
from coord_pub import CoordinatePublisher
import numpy as np  
import time


# callback function for motion capture frame
def receiveMoCapFrame(frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                      labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged):
    print("timestamp: ", timestamp, "\n ")


# callback function for rigid body frame
def receiveRigidBodyFrame(Qs, publisher, id, position, rotation):
    pos = np.asarray(position)
    Qs[id].put(pos)
    #print('LIFO ID:' + str(lifo[id]))
    #if timer >= 0.1:
        publisher.publish(id, position)
        #if id == 0:
            #print('TIME: ' + str(time.time()))

    
def tracking(Qs, publisher):
    try:
        # Initialize client object
        streamingClient = NatNetClient.NatNetClient(Qs, publisher)
        # Set client to read frames
        streamingClient.newFrameListener = receiveMoCapFrame
        streamingClient.rigidBodyListener = receiveRigidBodyFrame
        # Run client
        streamingClient.run()
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
                    print('time-interval' + str(et - st))
                    #print('pos: ' + str(pos))
    except KeyboardInterrupt:
        print('interrupted!')
        

def clear_q(queue):
    while not queue.empty():
        try:
            queue.get(False)
        except Empty:
            continue
        queue.task_done()


class MyManager(BaseManager):
    pass
MyManager.register('LifoQueue', LifoQueue)


if __name__ == '__main__':
    # initialize manager
    manager = MyManager()
    manager.start()

    # get number of robots from temporary plot and create n lifo queues
    plot = Live_Plot()
    plot.close()
    n = plot.num_robots

    # tuple of LifoQueues
    Qs = ()
    for id in range(n):
        locals()['lifo' + str(id)] = manager.LifoQueue(maxsize=10)
        Qs = Qs + (locals()['lifo' + str(id)],)
    print(Qs)

    #initialize coord publisher
    publisher = CoordinatePublisher(n)
    
    # create new processes
    track_process = Process(target=tracking, args=(Qs, publisher))
    plot_process = Process(target=plotting, args=(Qs))
  
    # run processes
    track_process.start()
    plot_process.start()
    
    # wait until processes finish
    track_process.join()
    plot_process.join()
    manager.shutdown()
    