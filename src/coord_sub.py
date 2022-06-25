import rospy
from std_msgs.msg import Float32MultiArray


class CoordinateSubscriber:


	def __init__(self):
		# initialize node
		rospy.init_node('coordinate_subscriber')
		# define robot/tracker id
		tracker_id = 0
		# topic to subscribe to, name of callback function
		self.sub = rospy.Subscriber("robot" + str(tracker_id), Float32MultiArray, self.log_coord)
		rospy.spin()


	def log_coord(self, new_coord):
		rospy.loginfo(str(new_coord.data))


if __name__ == '__main__':
	test_sub = CoordinateSubscriber()