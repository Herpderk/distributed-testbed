import rospy
from std_msgs.msg import Float32MultiArray
import ros_np_multiarray as rnm
import numpy as np
import time
from differential_controller import DifferentialLQR


class ControllerSubscriber:


	def __init__(self):
		# define robot/tracker id
		tracker_id = 0
		# initialize controller and variables for path tracking
		self.controller = DifferentialLQR(radius=64.5)
		self.path_index = 0
		self.error_lim = 50
		self.vel = np.zeros(2)
		# initialize node
		rospy.init_node('coord_sub_' + str(tracker_id))
		sub = rospy.Subscriber("robot" + str(tracker_id), Float32MultiArray, self.path_track)
		self.timer = time.time()
		rospy.spin()


	def path_track(self, new_coord):
		dt = time.time() - self.timer
		if dt >= 0.1:
			self.timer = time.time()
			self.controller.time_step = dt
			#st = time.time()
			rospy.loginfo(str(new_coord.data))

			#update current position and path
			pos = rnm.to_numpy_f32(new_coord.data[0])
			path = rnm.to_numpy_f32(new_coord.data[1])

			#calc error
			error = wp - pos
			error_mag = np.sqrt(np.einsum('i,i', error, error))

			# perform control action if outside of acceptable error
			if error_mag > self.error_lim:
			    pos, self.vel = self.controller.lqr_steer(wp, pos, self.vel)
			    if abs(self.vel[0]) < 10:
                        error_lim += error_lim

			    print('runtime per loop: ' + str(self.controller.time_step))
			    print()
			# else move on to next waypoint
			else:
				self.error_lim = 50
			    self.path_index += 1


if __name__ == '__main__':
	test_sub = ControllerSubscriber()