import matplotlib.pyplot as plt
import numpy as np
import random as r
import math
from sim.plot import plot, print_particle_error


AUTORUN = False
robot_start = 7
num_particles = 20
distance = 40
poles = [10, 15, 17, 19, 30, 39]


class Robot:
    def __init__(self, pos):
        self.pos = pos        
        self.move_dist = 1
        self.pole_dist = -100
        self.max_measurement = 3
        self.theta = np.array(poles)
        self.speed_sigma = np.sum(poles)
        self.theta_dot_sigma = np.dot(self.theta, self.speed_sigma)

    # Movement is perfectly accurate, even though we are assuming it isn't.
    def move(self):
        self.pos += self.move_dist

    def move(self, speed, theta_dot):
        self.theta += theta_dot
        self.x += math.cos(self.theta) * speed
        self.y += math.sin(self.theta) * speed

    def predict(self, speed, theta_dot):
        theta_dot = r.normalvariate(theta_dot, self.theta_dot_sigma)
        speed = r.normalvariate(speed, self.speed_sigma)
        self.move(speed, theta_dot)

    # Measurement is perfectly accurate even though we are assuming it isn't.
    def measure(self, poles):
        closest = min([pole - self.pos if (pole - self.pos >= 0) else (self.max_measurement + 1) for pole in poles])

        self.pole_dist = closest if closest <= self.max_measurement else -100

""" class Particle(Robot)
    1. Initialize constructor (weight, measurement_sigma)
    2. Set up a prediction method using normalization
    3. Set up probability density function to return the weight of the particle
    4. Update the weight using the probability density function
"""
class Particle(Robot):
    def __init__(self, pos):
        Robot.__init__(self, pos)
        self.weight = 0
        self.measurement_sigma = 0.3

    def predict(self):
        self.pos = np.random.normal(self.pos + self.move_dist, self.measurement_sigma)

    def probability_density_function(self, mu, x):
        weight = np.exp((-1/2)*((x - mu)/self.measurement_sigma)**2)/(self.measurement_sigma * np.sqrt(2 * np.pi))
        return weight

    def update_weight(self, robot_dist):
        self.weight = self.probability_density_function(robot_dist, self.pole_dist)

"""resample_particles(particles)
    Calculate the weight sum and return the resampled particles.
"""
def resample_particles(particles):
    # Potentially resample uniformly if weights are so low.
    weights = [part.weight for part in particles]

    weight_sum = sum(weights)
    if weight_sum < 0.05:
        resampled_particles = [Particle(r.uniform(0, distance)) for i in range(num_particles)]
    else:
        resample = r.choices(population=range(num_particles), weights=weights, k=num_particles)
        resampled_particles = [Particle(particles[i].pos) for i in resample]

    return resampled_particles



robot = Robot(robot_start)

# Setup particles.
particles = [Particle(r.uniform(0, distance - 1 )) for i in range(num_particles)]

# Plot starting distribution, no beliefs
plot(particles, poles, robot.pos)

# Begin Calculating
for j in range(39 - robot.pos):
    # Move
    if j != 0:
        robot.move()
        for particle in particles:
            particle.predict()

    # Measure
    robot.measure(poles)
    for particle in particles:
        particle.measure(poles)

        # Update Beliefs
        particle.update_weight(robot.pole_dist)

    print_particle_error(robot, particles)

    # Resample
    resampled_particles = resample_particles(particles)
    plot(particles, poles, robot.pos, resampled_particles, j, AUTORUN)
    particles = resampled_particles

plot(particles, poles, robot.pos, resampled_particles)
