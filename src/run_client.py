# Pulls localization data from Motive Tracker's server and publishes to ROS topics.
# Utilizes multiprocessing to simultaneously publish and update a 2D matplotlib visualization of the system.
# The number of agents can be changed in live_plot.py, tracker ids should be a range of integers from 0 to n.

from multiprocess import Process
from multiprocess.managers import BaseManager
from queue import LifoQueue
import numpy as np  
import time
from NatNetClient import NatNetClient
from live_plot import LivePlot
import example_paths


# callback function for motion capture frame
def receiveMoCapFrame(frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                      labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged):
    print("timestamp: ", timestamp, "\n ")


# callback function for rigid body frame
def receiveRigidBodyFrame(Qs, publisher, id, position, rotation):
    pos = np.asarray(position)
    Qs[id].put(pos)
    #if timer >= 0.1:
    publisher.publish(id, pos)
        #if id == 0:
        #    print('TIME: ' + str(time.time()))

    
def tracking(Qs, num_robots):
    try:
        # Initialize client object
        streamingClient = NatNetClient(Qs, num_robots)
        # Set client to read frames
        streamingClient.newFrameListener = receiveMoCapFrame
        streamingClient.rigidBodyListener = receiveRigidBodyFrame
        # Run client
        streamingClient.run()
    except KeyboardInterrupt:
        print('interrupted!')


def plotting(Qs, waypoints):
    new_plot = LivePlot(waypoints)
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

    #input example sinusoid path
    waypoints = example_paths.circle(3, 10000)

    # get number of robots from temporary plot and create n lifo queues
    plot = LivePlot(waypoints)
    plot.close()
    n = plot.num_robots

    # tuple of LifoQueues
    Qs = ()
    for id in range(n):
        locals()['lifo' + str(id)] = manager.LifoQueue(maxsize=20)
        Qs = Qs + (locals()['lifo' + str(id)],)
    print(Qs)
    
    # create new processes
    track_process = Process(target=tracking, args=(Qs, n))
    plot_process = Process(target=plotting, args=(Qs, waypoints))
  
    # run processes
    track_process.start()
    plot_process.start()
    
    # wait until processes finish
    track_process.join()
    plot_process.join()
    manager.shutdown()
    