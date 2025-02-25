import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]

options['DRIVE_IN_CIRCLE'] = False
# If False, measurements will be x,y.
# If True, measurements will be x,y, and current angle of the car.
# Required if you want to pass the driving in circle.
options['MEASURE_ANGLE'] = False
options['RECIEVE_INPUTS'] = False

""" Kalman Filter Class
    Matrices: initial state, uncertainty matrix, next state matrix, measurement matrix,
    measurement uncertainty, and identity matrix. Same as the 1D Kalman Filter, but expanded.
    Everything is scaled up to 4 dimensions.
    def predict(self, dt)
    def measure_and_update(self, measurements, dt)
    def recieve_inputs(self, u_steer, u_pedal)

    Kalman filter works in 1 dimension. Utilizes linear algebra to allow the car to drive forward.
    1. Start with 5 matrices, designed by the creator of the Kalman Filter.
        -x, u, P, F, H, R, I
        -x = State vector
        -u = External force
        -P = Uncertainty Matrix
        -F = State Transition Matrix
        -H = Measurement Matrix
        -R = Measurement Uncertainty
        -I = Identity Matrix
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
        # Initial State
        self.x = np.matrix([[0.],
                            [0.],
                            [0.],
                            [0.]])
        
        # External Force
        self.u = np.matrix([[0.],
                           [0.],
                           [0.],
                           [0.]])

        # Uncertainity Matrix
        self.P = np.matrix([[1000., 0., 0., 0.],
                            [0., 1000., 0., 0.],
                            [0., 0., 1000., 0.],
                            [0., 0., 0., 1000.]])

        # Next State Function
        self.F = np.matrix([[1., 0., 0.1, 0.],
                            [0., 1., 0., 0.1],
                            [0., 0., 1., 0.],
                            [0., 0., 0., 1.]])

        # Measurement Function
        self.H = np.matrix([[1., 0., 0., 0.],
                            [0., 1., 0., 0.]])

        # Measurement Uncertainty
        self.R = np.matrix([[0.0]])

        # Identity Matrix
        self.I = np.matrix([[1., 0., 0., 0.],
                            [0., 1., 0., 0.],
                            [0., 0., 1., 0.],
                            [0., 0., 0., 1.]])

    def predict(self, dt):
        # Increment the P matrix.
        self.P[0,0] += 0.1
        self.P[1,1] += 0.1
        self.P[2,2] += 0.1
        self.P[3,3] += 0.1
        # Calculate the new x and P matrices using the equations in the description.
        self.x = self.F*self.x + self.u
        self.P = self.F * self.P * np.transpose(self.F)
        return
        
    def measure_and_update(self,measurements, dt):
        # Create a matrix based on the measurements.
        Z = np.matrix(measurements)
        # y = flip z and multiply H * x
        y = np.transpose(Z) - (self.H * self.x)
        # S = H * P * flip H + r
        S = self.H * self.P * np.transpose(self.H) + self.R
        # K = P * flip H * inverted S
        K = self.P * np.transpose(self.H) * np.linalg.inv(S)

        # Various calculations to calculate x and P
        self.x = self.x + (K * y)
        self.P = (self.I - (K * self.H)) * self.P
        
        # Return first 2 values of the state of the matrix
        return [self.x[0], self.x[1]]

    def recieve_inputs(self, u_steer, u_pedal):
        # Function to recieve inputs
        return

sim_run(options,KalmanFilter)
