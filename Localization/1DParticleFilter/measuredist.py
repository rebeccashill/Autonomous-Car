import matplotlib.pyplot as plt
import math
import numpy as np
import random as r


# Measure the distance between the robot's position and the poles.
class Robot:
    def __init__(self, pos):
        self.pos = pos
        self.pole_dist = -100
        self.max_measurement = 3

    # Measurement is perfectly accurate even though we are assuming it isn't.
    def measure(self, poles):
        ### START STUDENT CODE
        # Set self.pole_dist to the distance to the closest pole.
        closest = min([ pole - self.pos if (pole - self.pos >= 0) else (self.max_measurement + 1) for pole in poles])

        self.pole_dist = closest if closest <= self.max_measurement  else -100
    
        ### END STUDENT CODE

class Particle(Robot):
    def __init__(self, pos):
        Robot.__init__(self, pos)



poles = [1, 10]
particle = Particle(0.1)
particle.measure(poles)
print("Output Should Be 0.9")
print("You Calculated: " + str(round(particle.pole_dist, 1)))
print()

particle.pos = 11
particle.measure(poles)
print("Output Should Be -100")
print("You Calculated: " + str(round(particle.pole_dist, 1)))
print()

particle.pos = 6.9
particle.measure(poles)
print("Output Should Be -100")
print("You Calculated: " + str(round(particle.pole_dist, 1)))
print()

particle.pos = 7.1
particle.measure(poles)
print("Output Should Be 2.9")
print("You Calculated: " + str(round(particle.pole_dist, 1)))
print()

particle.pos = 9.5
particle.measure(poles)
print("Output Should Be 0.5")
print("You Calculated: " + str(round(particle.pole_dist, 1)))
print()
