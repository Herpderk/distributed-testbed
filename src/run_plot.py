import rospy
from std_msgs.msg import Float32MultiArray
import time
import plot
import numpy as np
import random
from multiprocessing import Process


class CoordinateSubscriber:


	def __init__(self):
		# initialize plot object
		self.live_plot = plot.Plot_3D(1)
		# initialize node
		rospy.init_node('plot_subscriber')
		# define robot/tracker id
		self.tracker_id = 0
		

	def start_sub(self):
		# topic to subscribe to, name of callback function
		self.sub = rospy.Subscriber("robot" + str(self.tracker_id), Float32MultiArray, self.log_coord)
		rospy.spin()

	def log_coord(self, new_coord):
		#rospy.loginfo(str(new_coord.data))
		self.live_plot.x_coords[0] = new_coord.data[0]
		self.live_plot.y_coords[0] = new_coord.data[1]
		print([self.live_plot.x_coords[0], self.live_plot.y_coords[0]])
		#self.live_plot.update_plot(0, 1)

	def updating(self):
		self.live_plot.update_plot(0, 1)


if __name__ == '__main__':
	test_sub = CoordinateSubscriber()
	
	p1 = Process(target = test_sub.start_sub())
	p2 = Process(target = test_sub.updating())
	p1.start()
	p2.start()
	p1.join()
	p2.join()
   	
	
	