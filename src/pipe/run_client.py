import multiprocessing
from NatNetClient import NatNetClient
from live_plot import Live_Plot
import coord_pub
import numpy as np
import time
import numpy as np

# callback function for motion capture frame
def receiveMoCapFrame(frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                      labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged):
    print("timestamp: ", timestamp, "\n ")

# callback function for rigid body frame
def receiveRigidBodyFrame(conn, publisher, id, position, rotation ):
    #time.sleep(0.03)
    #print("pos of robot", id, " ", position, "\n ")
    publisher.publish(id, position)
    conn.send(np.array([id, position]))
    

def sending(conn):
    # Initialize client object
    streamingClient = NatNetClient(conn)
    # Set client to read motion capture frames (we won't be using this for now)
    streamingClient.newFrameListener = receiveMoCapFrame
    # Set client to read rigid body frames
    streamingClient.rigidBodyListener = receiveRigidBodyFrame
    # Run client
    streamingClient.run()

def plotting(conn):
    new_plot = Live_Plot()
    while True:
        st = time.time()
        list = conn.recv()
        id = list[0]
        pos = list[1]
        new_plot.update(id , pos)
        et = time.time()
        print(et - st)

if __name__ == '__main__':
    # creating a pipe
    parent_conn, child_conn = multiprocessing.Pipe()
    
    # creating new processes
    send_process = multiprocessing.Process(target=sending, args=(parent_conn,))
    plot_process = multiprocessing.Process(target=plotting, args=(child_conn,))
  
    # running processes
    send_process.start()
    plot_process.start()
  
    # wait until processes finish
    send_process.join()
    plot_process.join()
