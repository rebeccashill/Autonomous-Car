import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['CONSTANT_SPEED'] = True

""" Kalman Filter Class
    Matrices: initial state, uncertainty matrix, next state matrix, measurement matrix,
    measurement uncertainty, and identity matrix.
    def predict(self,t)
    def measure_and_update(self,measurements,t)

    Kalman filter works in 1 dimension. Utilizes linear algebra to allow the car to drive forward.
    1. Start with 5 matrices, designed by the creator of the Kalman Filter.
        -x, P, F, H, R
        -x = State vector
        -P = Uncertainty Matrix
        -F = State Transition Matrix
        -H = Measurement Matrix
        -R = Measurement Uncertainty
    2. Use these matrices to predict using the following formulas:
        -x = F x
        -P = F P F^T
    3. Measure and Update
        -Z = measurements
        -y = Z^T - H x
        -S = HPH^T + R
        -K = PH^T S^-1
        -x = x + K y
        -P = (I-KH)P

"""
class KalmanFilter:
    def __init__(self):
        self.v = 0
        self.prev_time = 0
        # Initial State
        self.x = np.matrix([[0.],
                            [0.]])

        # Uncertainty Matrix
        self.P = np.matrix([[1000., 0.],
                            [0., 1000.]])

        # Next State Function
        self.F = np.matrix([[1., 1.],
                            [0., 1.]])

        # Measurement Function
        self.H = np.matrix([[1., 0.]])

        # Measurement Uncertainty
        self.R = np.matrix([[0.01]])

        # Identity Matrix
        # The matrix we will use. 
        self.I = np.matrix([[1., 0.],
                            [0., 1.]])

    def predict(self,t):
        # Calculate dt.
        dt = t - self.prev_time
        # Dt gets put into the state transition matrix.
        self.F[0,1] = dt
        self.x = self.F*self.x
        # Transpose: F^T, flipping the matrix
        self.P =  self.F * self.P * np.transpose(self.F)
        return self.x[0,0]

    def measure_and_update(self,measurements,t):
        # Calculate dt.
        dt = t - self.prev_time
        # Dt gets put into the state transition matrix.
        self.F[0,1] = dt
        # Create a matrix based on the measurements.
        Z = np.matrix(measurements)
        # y = flip z and multiply H * x
        y = np.transpose(Z) - (self.H * self.x)
        # S = H * P * flip H + r
        S = self.H * self.P * np.transpose(self.H) + self.R
        # P * flip H * inverted S
        K =  self.P * np.transpose(self.H) * np.linalg.inv(S)

        # Various calculations to calculate x and P
        self.x = self.x + (K * y)
        self.P = (self.I - (K * self.H)) * self.P
        self.P[0,0] += 0.1
        self.P[1,1] += 0.1

        #Update previous time, update v, return
        self.v = self.x[1,0]
        self.prev_time = t
        return


sim_run(options,KalmanFilter)
