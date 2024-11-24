import numpy as np
import math
import time

import matplotlib.pyplot as plt

class Platform:

    at = 0                    # Tangential acceleration
    an = 0                   # Normal acceleration
    acc_last_update_time = 0

    dt = 0.5                  # Trajectory Frequency
    last_time = 0
    origin = np.zeros((3,))         # Initial Positions
    vel_init = np.zeros((3,))

    radius_earth = 6378.137e3

    A = np.array([[0, 1, 0, 0], [0, 0, 0, 1], [0, 0, 0, 0], [0, 0, 0, 0]])

    def __init__(self) -> None:
        pass

    def spawn(self, init_pos, init_vel):
        """
        Initializes the position and velocity of the platform
        """

        self.origin = init_pos
        self.vel_init = init_vel

    def curvilinear_motion(self, x_prev):
        """
        Generates next step in the trajectory using curvilinear motion

        Args:
            x_prev (6x1 numpy array) - previous time position and velocity
        Returns:
            x_curr (6x1 numpy array) - current time position and velocity
        """
        
        theta = math.atan2(x_prev[3],x_prev[4])
        x_curr = np.zeros((6,))
        now = time.time()

        if(self.last_time == 0):
            self.dt = 0
        else:
            self.dt = now - self.last_time

        omega = self.an/np.linalg.norm([x_prev[3], x_prev[4]])
        Ad, Bd = self.model(theta, omega)

        u = np.array([self.at, self.an])
        x = np.dot(Ad[0,:], np.array([x_prev[0], x_prev[3], x_prev[1], x_prev[4]])) + Bd[0,:]@u.transpose()
        x_dot = np.dot(Ad[1,:], np.array([x_prev[0], x_prev[3], x_prev[1], x_prev[4]])) +  Bd[1,:]@u.transpose()

        y = np.dot(Ad[2,:], np.array([x_prev[0], x_prev[3], x_prev[1], x_prev[4]])) +  Bd[2,:]@u.transpose()
        y_dot = np.dot(Ad[3,:], np.array([x_prev[0], x_prev[3], x_prev[1], x_prev[4]])) + Bd[3,:]@u.transpose()

        x_curr[0] = x
        x_curr[1] = y
        x_curr[3] = x_dot
        x_curr[4] = y_dot

        self.last_time = now

        return x_curr
    
    def model(self, phi, omega):
        """
        Returns the state matrices for discrete time curvilinear motion model
        """

        if omega != 0 and not np.isnan(omega):
                G11 = self.dt*np.cos(phi)/omega + np.sin(phi)/omega**2 - np.sin(phi + omega*self.dt)/omega**2
                G12 = np.cos(phi)/omega**2 - self.dt*np.sin(phi)/omega - np.cos(phi + omega*self.dt)/omega**2
                G21 = np.cos(phi)/omega - np.cos(phi + omega*self.dt)/omega
                G22 = np.sin(phi + omega*self.dt)/omega - np.sin(phi)/omega
            
        # After applying L'Hopital Rule to the given G matrix
        if omega == 0:
            G11 = self.dt**2*(np.sin(omega*self.dt+phi))/2
            G12 = self.dt**2*(np.cos(omega*self.dt+phi))/2
            G21 = self.dt*(np.sin(omega*self.dt+phi))
            G22 = self.dt*(np.cos(omega*self.dt+phi))

        G = np.array([[G11, G12], [G21, G22], [G12, -G11], [G22, -G21]])

        F = np.array([[1, self.dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, self.dt],  [0, 0, 0, 1]])

        return F,G

    def move(self, prev_pos, prev_vel):
        """
        Generates the platform trajectory based on previous step position and velocity
        """

        x_prev = (prev_pos - self.origin)*111320*1e-7
        x_curr = self.curvilinear_motion(np.hstack((x_prev, prev_vel)))

        # x_curr[0:3] = x_curr[0:3] + self.position_error()

        dx = x_curr[0:3]
        x_curr[0:3] = (dx/111320)*1e7 + self.origin

        return x_curr
    
    def lat_lng_to_meters(self, dlat: float, dlng: float):
        """
        Converts the change in lat and lng into meters

        Args:
            dlat (degE7) - Change in latitude with respect to an origin
            dlng (degE7) - Change in longitude with respect to an origin
        
        Return:
            dx and dy corresponding to dlat and dlng 
        """

        scaling_factor = 2*np.pi*self.radius_earth/360
        dx = dlat*1e-7*scaling_factor
        dy = dlng*1e-7*(np.cos(np.deg2rad(self.origin[0])))*scaling_factor

        return np.array([dx, dy])
    
    def meters_to_lat_lng(self, dx: float, dy: float):
        """
        Converts change in position to change in lat and lng

        Args:
            dx - Change in x direction (along North) with respect to origin
            dy - Change in y direction (along East) with respect to origin

        Results:
            dlat (degE7) and dlng (degE7) corresponding to dx and dy
        """

        scaling_factor = 2*np.pi*self.radius_earth/360
        dlat = dx*1e7/scaling_factor
        dlng = dy*1e7/((np.cos(np.deg2rad(self.origin[0])))*scaling_factor)

        return np.array([dlat, dlng])

    
    def position_error(self, noise_info):

        match(noise_info['type']):
            case 'Gaussian':
                mu = noise_info['mu']
                sigma = noise_info['sigma']
                pos_error = sigma*np.random.randn(6,) + mu
                pos_error[0:2] = self.meters_to_lat_lng(pos_error[0], pos_error[1])
        
                return pos_error
    
    def save_trajectory_data(self, x_curr, filename):
         # Append to CSV using numpy
        with open(filename, 'ab') as f:
            np.savetxt(f, [x_curr], delimiter=',', fmt='%f')

if __name__ == "__main__":

    platform = Platform()

    platform.an = 2.5
    platform.at = 0
    platform.vel_init = np.array([5, 0, 0])

    N = 1000
    x = np.zeros((N,6))
    platform.dt = 0.01

    for i in range(0, N, 1):

        if i == 0:
            x[i, 0:3] = np.array([129920050, 802368490, 0])
            x[i, 3:6] = platform.vel_init
            continue

        x[i, :] = platform.move(x[i-1, 0:3], x[i-1, 3:6])

    for i in range(0, len(x), 1):

        xn = x[i,:]
        xn[0:3] = x[i, 0:3] + platform.position_error() 
        platform.save_trajectory_data(xn, 'target_noisy.csv')
    plt.figure()
    plt.plot(x[:,0], x[:, 1])
    plt.title("Trajectory")

    plt.figure()
    plt.subplot(2,1,1)
    plt.plot(x[:,0])
    plt.title("x")

    plt.subplot(2,1,2)    
    plt.plot(x[:,1])
    plt.title("y")

    plt.figure()
    plt.subplot(2,1,1)
    plt.plot(x[:,3])
    plt.title("vx")
    
    plt.subplot(2,1,2)
    plt.plot(x[:,4])
    plt.title("vy")

    plt.show()
    
