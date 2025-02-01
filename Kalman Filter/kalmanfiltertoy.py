import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['CONSTANT_SPEED'] = True

class KalmanFilterToy:
    def __init__(self):
        self.v = 0
        self.prev_x = 0
        self.prev_time = 0

    def predict(self,t):
        return self.v*(t - self.prev_time) + self.prev_x
    
    def measure_and_update(self,x,t):
        measured_v = (x - self.prev_x)/(t - self.prev_time)
        self.v += 0.5*(measured_v - self.v)

        self.prev_x = x
        self.prev_time = t
        return


sim_run(options,KalmanFilterToy)
