from multiprocess import Process 
from multiprocess import Manager
#from multiprocessing import Process
import NatNetClient
import plot
import coord_pub
import time

# callback function for motion capture frame
def receiveMoCapFrame(frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                      labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged):
    print("timestamp: ", timestamp, "\n ")


# callback function for rigid body frame
def receiveRigidBodyFrame(lifo, timer, publisher, id, position, rotation ):
    print(type(position))
    lifo[id].append(position)
    if timer >= 0.1:
        publisher.publish(id, position)
        if id == 0:
            print('TIME: ' + str(time.time()))

    

def tracking(lifo):
    # Initialize client object
    streamingClient = NatNetClient.NatNetClient(lifo)
    # Set client to read motion capture frames (we won't be using this for now)
    streamingClient.newFrameListener = receiveMoCapFrame
    # Set client to read rigid body frames
    streamingClient.rigidBodyListener = receiveRigidBodyFrame
    # Run client
    streamingClient.run()


def plotting(lifo, new_plot):
    while True:
        #st = time.time()
        if len(lifo) > 0:
            for id in range(len(lifo)):
                arr = lifo.pop()
                del lifo[id][:]
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
    for i in range(n):
        fake_lifo.append([])

    
    # creating new processes
    track_process = Process(target=tracking, args=[fake_lifo])
    plot_process = Process(target=plotting, args=[fake_lifo, new_plot])
  
    # running processes
    track_process.start()
    plot_process.start()
    
    # wait until processes finish
    track_process.join()
    plot_process.join()
    
    