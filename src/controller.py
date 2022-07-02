import matplotlib.pyplot as plt
import numpy as np
import jax
import jax.numpy as jnp
from jax import grad, jit, vmap
import time

class MPC_Controller:


	def __init__(self):
		self.deltaT = 1 #s
		self.wheel_radius = 64.5 #mm
		self.bodycenter_X = 46.5 #mm
		self.bodycenter_Y = 93.0 #mm


	def angular_vels (self, v0, v1):
		deltaTheta = jnp.arccos((jnp.dot(v0, v1)) / (jnp.sqrt(v0.dot(v0))*jnp.sqrt(v1.dot(v1))))
		vel_matrix = (1/self.deltaT) * jnp.array([v0[0], v1[1], deltaTheta])
		dynamics_matrix = jnp.array([[1, -1, -(self.bodycenter_X+self.bodycenter_Y)],
									 [1,  1,  (self.bodycenter_X+self.bodycenter_Y)],
									 [1,  1, -(self.bodycenter_X+self.bodycenter_Y)],
									 [1, -1,  (self.bodycenter_X+self.bodycenter_Y)]])
		ang_vel_matrix = (1/self.wheel_radius) * jnp.matmul(dynamics_matrix, vel_matrix)

		return ang_vel_matrix


if __name__ == '__main__':
	new_controller = MPC_Controller()

	v0 = jnp.array([10, 2])
	v1 = jnp.array([5, 13])

	new_controller.angular_vels(v0,v1)

	angular_vels_jit = jax.jit(new_controller.angular_vels)
	print(angular_vels_jit(v0,v1))