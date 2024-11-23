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

# Coordinate 1 - Life Pod 
x1 = 0.0 
y1 = 0.0 

# Coordinate 2 - Gun Island Beach 
x2 = 292.1843 
y2 = 1165.9470 

# Coordinate 3 - Auora Entrance 
x3 = 1077.8830 
y3 = 214.4043 

# Initial guesses for finding the location of a point of interest 
Pg1 = ((x1 + x2 + x3) / 2, (y1 + y2 + y3) / 2) 
Pg2 = (-Pg1[0], -Pg1[1]) 

# Known coordinates 
Pi = \
(
    (x1, y1, "Life Pod"),        # Coordinate 1 - Life Pod 
    (x2, y2, "Gun Island"),      # Coordinate 2 - Gun Island Beach 
    (x3, y3, "Auora Entrance")   # Coordinate 3 - Auora Entrance 
)

# Location (distances) of points of interest 
di = \
(
    # Wrecks 
    (449, 1489, 1535, Pg1, "W0"), 
    (1275, 2346, 2258, Pg1, "W1"), 
    (745, 1918, 1616, Pg1, "W2"), 
    # Caves 
    # Materials 
    # Islands 
    # (1203, 0, 1234, "I0"),         # Gun Island - already set as a landmark 
    (1177, 2334, 2122, Pg1, "I1"),   # Habitat Island 
    # Habitats 
    # Discoveries 
    (1388, 2274, 2384, Pg2, "D0"),   # Alien Research Lab (Orange tablet) & Fossil 
    (1629, 986, 831, Pg1, "D1"),     # Entrance to underground network (depth ~400m) 
)

# List of locations for points of interest (populated later) 
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
    d_tolerance = 5   # To account for (minor) human error 

    d12 = planer_distance(x1, y1, x2, y2) 
    d13 = planer_distance(x1, y1, x3, y3) 
    d23 = planer_distance(x2, y2, x3, y3) 

    if ((d12 > (d1n + d2n + d_tolerance)) or 
        (d13 > (d1n + d3n + d_tolerance)) or 
        (d23 > (d2n + d3n + d_tolerance))): 
        print("Distances don't have a solution.") 
        error = 1 

    return error 

#================================================================================


#================================================================================
# Application 

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
        d[3],                                        # The initial guess 
        args=(Pi, d),                                # Additional parameters for mse 
        method='L-BFGS-B',                           # The optimisation algorithm 
        options={ 'ftol': 1e-5, 'maxiter': 1e+7 })   # Tolerance & max iterations 

    Pn.append([result.x, d[4]]) 


# Set up plot 
fig, ax = plt.subplots() 

# Plot the reference points (landmarks) 
for P in Pi: 
    ax.scatter(P[0], P[1], s=10, c='#ff7f0e') 
    ax.annotate(P[2], (P[0], P[1])) 

# Plot the approximate points 
for P in Pn: 
    ax.scatter(P[0][0], P[0][1], s=20, c='#17becf') 
    ax.annotate(P[1], (P[0][0], P[0][1])) 

ax.set_xlim(-2000, 2000) 
ax.set_ylim(-2000, 2000) 
ax.title.set_text("Subnautica") 
ax.set_xlabel("Latitude (m)") 
ax.set_ylabel("Longitude (m)") 
plt.show() 

#================================================================================
