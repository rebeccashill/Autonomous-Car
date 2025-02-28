from helpers import plot_poles, plot_measurement_circles2, plot_robot
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable


def distance_difference_squared(guess_location, pole, pole_measurements):
    # calculate the distance between the guess location and pole using the Euclidean distance formula
    output = np.linalg.norm(np.array(guess_location)-np.array(pole))
    # compare distance vs pole_measurement
    output = np.sqrt(output)
    # find the minimum of the differences between the output of the formula and the pole measurements
    diff1 = pole_measurements[0] - output
    diff2 = pole_measurements[1] - output
    diff3 = pole_measurements[2] - output
    output = min(diff1**2, diff2**2, diff3**2)
    return output**2

# 1D Example
# pole = 5
# pole_measurement = [3, 2, 7]
# guess_location = 2
# distance = 2


def total_cost(guess_location, poles, pole_measurements):
    total = 0
    total += distance_difference_squared(guess_location, poles[0], pole_measurements)
    total += distance_difference_squared(guess_location, poles[1], pole_measurements)
    total += distance_difference_squared(guess_location, poles[2], pole_measurements)
    return total

poles = [[0, 0]]
poles += [[3, 2]]
poles += [[7, 4]]
pole_measurements = [11.180339887498949]
pole_measurements += [7.615773105863909]
pole_measurements += [3.1622776601683795]

plot_poles(poles)
plt.title('Pole Location')
plt.show()

plot_poles(poles)
plot_measurement_circles2(poles, pole_measurements)
plt.title('Pole Measurements')
plt.show()

plot_poles(poles)
plot_measurement_circles2(poles, pole_measurements, zoom_out=True)
plt.title('Pole Measurements')
plt.show()

location_solution = minimize(total_cost, [0, 0], bounds=[[-20, 20], [-20, 20]], args=(poles, pole_measurements))
print(location_solution)

plot_poles(poles)
plot_measurement_circles2(poles, pole_measurements)
plot_robot(location_solution.x)
plt.title('Calculated Location')
plt.show()
