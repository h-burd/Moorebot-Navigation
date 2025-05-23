import numpy as np

class ExtendedKalmanFilter:
    """
    EKF implemented as seen in localization slides 
    """
    def __init__(self):
        self.state = np.zeros((3, 1)) # robot state [x, y, theta]
         
        self.P = np.array([[0.1, 0, 0],
                          [0, 0.1, 0],
                          [0, 0, 1]]) # covariance matrix
        
        SIGMA_V = 0.1
        SIGMA_W = 0.1

        self.Q = np.array([[SIGMA_V**2, 0, 0],
                            [0, SIGMA_V**2, 0],
                            [0, 0, SIGMA_W**2]])
        
        MEAS_NOISE = 1.5 # increasing this will make filter smoother but less responsive to measurement updates
        self.R = np.array([[MEAS_NOISE**2, 0, 0],
                            [0, MEAS_NOISE**2, 0],
                            [0, 0, MEAS_NOISE**2]]) # measurement noise covariance

        


    def predict(self, control_input):
        # control_input is a tuple (v, w) where v is the linear velocity and w is the angular velocity
        v, w = control_input
        dt = 0.1
        theta = self.state[2, 0]
        # State transition model
        self.state[0, 0] += v * np.cos(theta) * dt
        self.state[1, 0] += v * np.sin(theta) * dt
        self.state[2, 0] += w * dt
        self.state[2, 0] = (self.state[2, 0] + np.pi) % (2 * np.pi) - np.pi # Normalize theta 

        # Jacobian of the kinematic model
        F = np.array([[1, 0, -v * np.sin(theta) * dt],
                      [0, 1, v * np.cos(theta) * dt],
                      [0, 0, 1]])

        # Propagate the covariance
        self.P = F @ self.P @ F.T + self.Q

        return self.state
    
    def update(self, measurement):
        # measurement is a tuple (x, y, theta) of the observed position and orientation of the robot
        x, y, theta = measurement
        z = np.array([[x], [y], [theta]])

        # Measurement model
        H = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]]) # x,y,theta are directly updated

        # Kalman operations same as HW 3
        # Kalman gain
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        # Update the state and covariance
        self.state += K @ (z - H @ self.state)
        self.P = (np.eye(3) - K @ H) @ self.P

        return self.state, self.P
    