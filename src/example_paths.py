import numpy as np  


def sine_wave(size, num_points):
    waypoints = np.empty((0, 2), float)
    for t in range(num_points):
        waypoints = np.append(waypoints, [[t, 
            (size/2)*np.sin((5*(2*np.pi*t/num_points) - np.pi/2))+(size/2)]], axis = 0)
    return waypoints


def circle(size, num_points):
    waypoints = np.empty((0, 2), float)
    for t in range(num_points):
        waypoints = np.append(waypoints, [[(size/2) + (size/2)*np.cos(2*np.pi*t/num_points), 
                                           (size/2) + (size/2)*np.sin(2*np.pi*t/num_points)]], axis = 0)
    return waypoints


def lissajous(size, num_points):
    waypoints = np.empty((0, 2), float)
    for t in range(num_points):
        waypoints = np.append(waypoints, [[(size/2) + (size/2)*np.cos(7*2*np.pi*t/num_points), 
                             (size/2) + (size/2)*np.sin(5*2*np.pi*t/num_points)]], axis = 0)
    return 