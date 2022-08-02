# BACKGROUND
# A controller that combines LQR steering with PID speed control for optimal path tracking.
# The LQR's control policy is found using SciPy's solver for the discrete-time algebraic Riccati equation.
# The PID is provided by the simple-pid python package, and the setpoint of the speed adjusts for turn angles. 
# This controller is intended for 4-wheel mecanum drive systems.

# INSTRUCTIONS
# Initialize LQR_PID object with the wheel radius, width, height, and desired maximum velocity.
# Path must be an array of (x,y) arrays and inputted into gen_path to be usable for the controller.
# Run path_tracking, an example can be found in main. 

from scipy import linalg as la
from simple_pid import PID
import jax.numpy as jnp
import time


class LQR_PID:


    def __init__(self, radius, width, height, v, spd_ctrl):
        self.wheel_radius = radius # mm
        self.center_X = width/2    # mm
        self.center_Y = height/2   # mm
        self.max_V = v             # mm/s
        self.speed_control = spd_ctrl
        self.time_step = 0.1       # s
        self.K1 = self.K_matrix()


    # The solution to the optimal control policy u = K*x
    def K_matrix(self):
        A = jnp.eye(3)
        B = self.time_step*jnp.eye(3)
        Q = 1 * jnp.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
            ])
        #R punishes actuation
        R = 0.1 * jnp.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
            ])

        self.A1 = jnp.block([
            [A,                       B],
            [jnp.zeros((3,3)), jnp.eye(3)]
            ])
        self.B1 = jnp.block([
            [B        ],
            [jnp.eye(3)]
            ])
        Q1 = jnp.block([
            [Q,         jnp.zeros((3,3))],
            [jnp.zeros((3,3)),         R]
            ])
        #R1 punishes turning
        self.R1 = 4 * jnp.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
            ])

        P = la.solve_discrete_are(self.A1, self.B1, Q1, self.R1)
        K1 = la.inv(self.R1) @ self.B1.T @ P
        return K1


    # Continuously adjusts the setpoint of the pid based on turn angles in the horizon
    def pid_speed(self, pid, net_turn, curr_spd):
        if self.speed_control:
            if jnp.cos(net_turn) >= 0:
                    cost = abs(0.5*jnp.sin(net_turn))
            elif jnp.cos(net_turn) < 0:
                    cost = abs(0.5 - 0.5*jnp.cos(net_turn)) 

            ideal_spd = 90*self.max_V*((1 - cost)**8)
            pid.setpoint = ideal_spd
            
            if (curr_spd > 0.2*self.max_V) & (ideal_spd > 0.2*90*self.max_V):
                pid.reset()
                pid.tunings = (0.01, 0.0, 0.00)
            elif (curr_spd < 0.2*self.max_V) & (ideal_spd > 0.2*90*self.max_V):
                pid.reset()
                pid.tunings = (0.01, 0.005, 0.0)

            nxt_spd = pid(curr_spd, self.time_step)
            return nxt_spd
        else:
            nxt_spd = self.max_V
            return nxt_spd


    # Calculates u, inputs the pid speed, and calculates the next state A*x - B*u
    def lqr_steer(self, path_pos, pos, vel, spd):
        error = path_pos - pos
        state = jnp.block([error, vel])
        U = self.K1 @ state
        nxt_state = (self.A1 @ state) - (self.B1 @ U)

        nxt_err, nxt_vel = jnp.split(nxt_state, 2)
        nxt_pos = path_pos - nxt_err
        nxt_vel = spd * nxt_vel / jnp.sqrt(jnp.einsum('...i,...i', nxt_vel, nxt_vel))
        nxt_spd = jnp.sqrt(jnp.einsum('...i,...i', nxt_vel, nxt_vel))

        print('spd:' + str(nxt_spd))
        print('vel:' + str(nxt_vel))
        return nxt_pos, nxt_vel


    # Uses inverse kinematics to calculate the angular velocity of each wheel
    def motor_spds(self, vel):
        kinematics = jnp.array([
            [1, -1, -(self.center_X + self.center_Y)],
            [1,  1,  (self.center_X + self.center_Y)],
            [1,  1, -(self.center_X + self.center_Y)],
            [1, -1,  (self.center_X + self.center_Y)]
            ])

        ang_vel = (1/self.wheel_radius) * kinematics @ vel
        print('motor speeds: ' + str(ang_vel))
        return ang_vel


    # Adds angles between (x,y) waypoints to be used for steering
    def gen_path(self, waypoints):
        new_col = jnp.zeros((waypoints.shape[0], 1))
        for i in range(new_col.shape[0] - 1):
            x_diff = waypoints[i+1, 0] - waypoints[i, 0]
            y_diff = waypoints[i+1, 1] - waypoints[i, 1]
            angle = jnp.arctan2(y_diff, x_diff)
            new_col[i + 1] = (angle)
        
        path = jnp.append(waypoints, new_col, 1)
        return path


    # increase prediction horizon to decelerate earlier
    def path_tracking(self, path):
        pid = PID(0.04, 0.0, 0.0, setpoint = 90*self.max_V)
        pid.output_limits = (0, self.max_V)
        
        pos = path[0] 
        vel = jnp.array([0, 0, 0])   
        horizon = 7

        for i in range (jnp.size(path, 0) - horizon):
            if jnp.size(path, 0) - i < horizon:
        	    horizon = jnp.size(path, 0) - i

            wp = path[i]
            spd = 0

            while True:
                st = time.time()

                error = wp - pos
                error_mag = jnp.sqrt(jnp.einsum('i,i', error, error))
                error_lim = 50 #10.75*self.R1[0][0] + 0.2275*self.max_V - 45
            	# IMPORTANT: if error limit is too small, infinite oscillation occurs
                if error_mag > error_lim:
                    net_turn = path[i+horizon][2] - path[i][2]
                    spd = self.pid_speed(pid, net_turn, spd)

                    pos, vel = self.lqr_steer(wp, pos, vel, spd)
                    ang_spds = self.motor_spds(vel)

                    time.sleep(0.094)
                    et = time.time()
                    print('runtime per loop: ' + str(et - st))
                    print()
                else:
                    break


# example path functions
def sine_wave(size, num_points):
    waypoints = jnp.empty((0, 2), float)
    for t in range(num_points):
    	waypoints = jnp.append(waypoints, [[t, 
    		(size/2)*jnp.sin((5*(2*jnp.pi*t/num_points) - jnp.pi/2))+(size/2)]], axis = 0)
    return waypoints


def circle(size, num_points):
    waypoints = jnp.empty((0, 2), float)
    for t in range(num_points):
    	waypoints = jnp.append(waypoints, [[(size/2) + (size/2)*jnp.cos(2*jnp.pi*t/num_points), 
    	    	                           (size/2) + (size/2)*jnp.sin(2*jnp.pi*t/num_points)]], axis = 0)
    return waypoints


def lissajous(size, num_points):
    waypoints = jnp.empty((0, 2), float)
    for t in range(num_points):
        waypoints = jnp.append(waypoints, [[(size/2) + (size/2)*jnp.cos(7*2*jnp.pi*t/num_points), 
    	                     (size/2) + (size/2)*jnp.sin(5*2*jnp.pi*t/num_points)]], axis = 0)
    return waypoints
    

if __name__ == "__main__":
    # wheel radius, width, height, and max velocity in terms of mm and s
    # to disable speed control (constant speed), enter False
    controller = LQR_PID(radius=64.5, width=46.5, height=93.0, v=300.0, spd_ctrl=True)
    # make sure path has points in the form of (x,y) or (x,y,theta)
    size = 1000
    num_points = 1000
    waypoints = lissajous(size, num_points)
    # if points are (x,y) then run it through gen_path
    path = controller.gen_path(waypoints)
    controller.path_tracking(path)
    