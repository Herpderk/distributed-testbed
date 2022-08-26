import rospy
from std_msgs.msg import Float32MultiArray
import ros_np_multiarray as rnm
import numpy as np
import time
from mecanum_controller import LQR_PID


class ControllerSubscriber:


	def __init__(self):
		# define robot/tracker id
		tracker_id = 0
		# initialize controller and variables for path tracking
		self.controller = LQR_PID(radius=64.5, width=46.5, height=93.0, v=300.0, spd_ctrl=True)
		self.path_index = 0
		self.horizon = 6
		self.error_lim = 50
		self.spd = 0
		self.vel = np.array([0, 0, 0])
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

			#decrease horizon if nearing end of path
			if np.size(path, 0) - self.path_index < self.horizon:
				self.horizon = np.size(path, 0) - self.path_index - 1
			wp = path[self.path_index]

			#calc error
			error = wp - pos
			error_mag = np.sqrt(np.einsum('i,i', error, error))

			# perform control action if outside of acceptable error
			if error_mag > self.error_lim:
			    net_turn = path[self.path_index + self.horizon][2] - path[self.path_index][2]
			    self.spd = self.controller.pid_speed(self.controller.pid, net_turn, self.spd)
			    self.vel = self.controller.lqr_steer(wp, pos, self.vel, self.spd)
			    spin_motors = self.controller.motor_spds(self.vel)
			    print('runtime per loop: ' + str(self.controller.time_step))
			    print()
		# else move on to next waypoint
		else:
		    self.path_index += 1


if __name__ == '__main__':
	test_sub = ControllerSubscriber()
