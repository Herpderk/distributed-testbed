import rospy
from std_msgs.msg import Float32MultiArray


class CoordinatePublisher:

    def __init__(self):
        # initialize node
        rospy.init_node('coordinate_publisher')
        # input the ids of each tracker
        tracker_ids = [0, 1, 2]
        for i in tracker_ids:
            self.pub = rospy.Publisher(
                "robot" + str(i), Float32MultiArray, queue_size=100)

    def publish(self, id, coordinate):
        # switch publisher to appropriate topic
        self.pub = rospy.Publisher(
            "robot" + str(id), Float32MultiArray, queue_size=100)
        # assign coordinates to message
        new_coord = Float32MultiArray()
        new_coord.data = coordinate
        # publish to topic
        self.pub.publish(new_coord)


if __name__ == '__main__':
    # test code
    publisher = CoordinatePublisher()
    while True:
        for i in range(3):
            publisher.publish(i, [1.6, 0.0, 3.49495858])
