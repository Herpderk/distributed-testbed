import rospy
from std_msgs.msg import Float32MultiArray
import mec_control_no_gui


class CoordinateSubscriber:


	def __init__(self):
		# define robot/tracker id
		tracker_id = 0
		# initialize node
		rospy.init_node('coord_sub' + str(tracker_id))
		# topic to subscribe to, name of callback function
		sub = rospy.Subscriber("robot" + str(tracker_id), Float32MultiArray, self.path_track)
		# initialize controller
		self.controller = LQR_PID(radius=64.5, width=46.5, height=93.0, v=300.0, spd_ctrl=True)
		# iterate thru path with index
		self.path_index = 0

		rospy.spin()


	def path_track(self, pos):
		rospy.loginfo(str(pos.data))
		



if __name__ == '__main__':
	test_sub = CoordinateSubscriber()