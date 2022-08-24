from multiprocess import Process 
from multiprocess import Manager
#from multiprocessing import Process
import NatNetClient
from live_plot import Live_Plot
import coord_pub
import time

# callback function for motion capture frame
def receiveMoCapFrame(frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                      labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged):
    print("timestamp: ", timestamp, "\n ")


# callback function for rigid body frame
def receiveRigidBodyFrame(lifo, timer, publisher, id, position, rotation ):
    lifo[id].append(position)
    #print('LIFO ID:' + str(lifo[id]))
    if timer >= 0.1:
        publisher.publish(id, position)
        if id == 0:
            pass
            #print('TIME: ' + str(time.time()))

    

def tracking(lifo):
    # Initialize client object
    streamingClient = NatNetClient.NatNetClient(lifo)
    # Set client to read motion capture frames (we won't be using this for now)
    streamingClient.newFrameListener = receiveMoCapFrame
    # Set client to read rigid body frames
    streamingClient.rigidBodyListener = receiveRigidBodyFrame
    # Run client
    streamingClient.run()


def plotting(lifo):
    new_plot = Live_Plot()
    while True:
        #st = time.time()
        for id in range(len(lifo)):
            if len(lifo[id]) > 0:
                arr = lifo.pop()
                #print(arr)
                del lifo[id][:]
                #print('ARRAY AFTER DELETION: ' + str(lifo[id]))
                pos = arr[0]
                print('pos: ' + str(pos))

                if id == 0:
                    print('PLOTTING: ' + str(pos))
                new_plot.update(id , pos)
        
  

if __name__ == '__main__':
    manager = Manager()
    fake_lifo = manager.list()

    plot = Live_Plot()
    fake_lifo = [ [ [] for i in range(1) ] for i in range(plot.num_robots) ]
    print('PSEUDO LIFO: ' + str(fake_lifo))
    for i in range(plot.num_robots):
        fake_lifo[i][0] = [0,0,0]

    
    # creating new processes
    track_process = Process(target=tracking, args=[fake_lifo])
    plot_process = Process(target=plotting, args=[fake_lifo])
  
    # running processes
    track_process.start()
    time.sleep(0.5)
    plot_process.start()
    
    # wait until processes finish
    track_process.join()
    plot_process.join()
    
    