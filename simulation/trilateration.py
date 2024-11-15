#================================================================================
# File: trilateration.py 
# 
# Author: Sam Donnelly (samueldonnelly11@gmail.com)
# 
# Description: Find location of a point given the points distances to 3 other 
#              known locations. 
# 
# Resources: 
# - https://www.alanzucconi.com/2017/03/13/positioning-and-trilateration/ 
#
# Date: November 12, 2024 
#================================================================================


#================================================================================
# Imports 

import sys 
import math 
from scipy.optimize import minimize 
import matplotlib.pyplot as plt 
import numpy as np 

#================================================================================


#================================================================================
# Reference points 

# Coordinate 1 
x1 = 0 
y1 = 0 

# Coordinate 2 
x2 = 1000 
y2 = 0 

# Coordinate 3 
x3 = 0 
y3 = 1000 

# Coordinate distances (updated later) 
d12 = 0 
d13 = 0 
d23 = 0 

d_tolerance = 2 

# Initial guess and list of known coordinates 
Pg = ((x1 + x2 + x3) / 2, (y1 + y2 + y3) / 2) 
Pi = ((x1, y1), (x2, y2), (x3, y3)) 

# Location (distances) of points of interest 
di = \
(
    # Wrecks 
    (707, 707, 707, "W0"), 
    (707, 1581, 1581, "W1"), 
    (707, 1581, 707, "W2"), 
    (707, 707, 1581, "W3"), 
    # # Caves 
    # (1, 1, 1, "C0"), 
    # (1, 1, 1, "C1"), 
    # (1, 1, 1, "C2"), 
    # # Materials 
    # (1, 1, 1, "M0"), 
    # (1, 1, 1, "M1"), 
    # (1, 1, 1, "M2"), 
    # # Islands 
    # (1, 1, 1, "I0"), 
    # (1, 1, 1, "I1"), 
    # (1, 1, 1, "I2"), 
    # # Habitats 
    # (1, 1, 1, "H0"), 
    # (1, 1, 1, "H1"), 
    # (1, 1, 1, "H2"), 
)

Pn = [] 

#================================================================================


#================================================================================
# Calculations 

# Planer distance 
def planer_distance(xa, ya, xb, yb): 
    return math.sqrt((xa - xb)**2 + (ya - yb)**2) 


# Mean Square Error 
# locations: [ (lat1, long1), ... ] 
# distances: [ distance1, ... ] 
def mse(x, locations, distances): 
    mse = 0.0 
    for location, distance in zip(locations, distances): 
        radius = planer_distance(x[0], x[1], location[0], location[1]) 
        mse += math.pow(distance - radius, 2.0) 
    return mse / len(distances) 

#================================================================================


#================================================================================
# Checks 

# Coordinate check 
def coordinate_check(): 
    error = 0 
    
    # Coordinates form a straight line 
    if ((x1 == x2 == x3) or (y1 == y2 == y3)): 
        print("Coordinates cannot form a straight line.") 
        error = 1 

    # Two or more coordinates are the exact same 
    if (((x1 == x2) and (y1 == y2)) or 
        ((x1 == x3) and (y1 == y3)) or 
        ((x2 == x3) and (y2 == y3))): 
        print("Coordinates cannot be the same.")
        error = 1 
    
    return error 


# Distance check 
def distance_check(d1n, d2n, d3n): 
    error = 0 

    if ((d12 > (d1n + d2n + d_tolerance)) or 
        (d13 > (d1n + d3n + d_tolerance)) or 
        (d23 > (d2n + d3n + d_tolerance))): 
        print("Distances don't have a solution.") 
        error = 1 

    return error 

#================================================================================


#================================================================================
# Application 

d12 = planer_distance(x1, y1, x2, y2) 
d13 = planer_distance(x1, y1, x3, y3) 
d23 = planer_distance(x2, y2, x3, y3) 

# Make sure the reference points are valid 
if (coordinate_check()): 
    sys.exit() 

# Find the approximate location of each point of interest 
for d in di: 
    # Check that the provided distances make sense 
    if (distance_check(d[0], d[1], d[2])): 
        continue 

    # Update the calculation location 
    result = minimize(
        mse,                                         # The error function
        Pg,                                          # The initial guess
        args=(Pi, d),                                # Additional parameters for mse
        method='L-BFGS-B',                           # The optimisation algorithm
        options={ 'ftol': 1e-5, 'maxiter': 1e+7 })   # Tolerance & max iterations 

    Pn.append([result.x, d[3]]) 


# Plot the reference points 
plt.scatter(Pi[0][0], Pi[0][1]) 
plt.scatter(Pi[1][0], Pi[1][1]) 
plt.scatter(Pi[2][0], Pi[2][1]) 

# Plot the approximate points 
for P in Pn: 
    plt.scatter(P[0][0], P[0][1]) 
plt.show() 

#================================================================================
