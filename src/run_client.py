# Uses the Python NatNetClient.py library to establish a connection (by creating a NatNetClient),
# and receive data via a NatNet connection and decode it using the NatNetClient library.
import time
from NatNetClient import NatNetClient
import coord_pub


# callback function for motion capture frame
def receiveMoCapFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    print("timestamp: ",timestamp,"\n ")


# callback function for rigid body frame
def receiveRigidBodyFrame( publisher, id, position, rotation ):
    print( "pos of robot", id, position,"\n ")
    publisher.publish(id, position)
    time.sleep(0.03)


if __name__ == '__main__':
    # Initialize client object
    streamingClient = NatNetClient()
    # Set client to read motion capture frames (we won't be using this for now)
    streamingClient.newFrameListener = receiveMoCapFrame
    # Set client to read rigid body frames
    streamingClient.rigidBodyListener = receiveRigidBodyFrame
    # Run client
    streamingClient.run()