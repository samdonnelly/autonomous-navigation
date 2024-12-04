#================================================================================
# File: trilateration.py 
# 
# Author: Sam Donnelly (samueldonnelly11@gmail.com)
# 
# Description: Find the location of a point given its distance to at least 3 
#              reference points. Note that this is for a 2D Cartesian coordinate 
#              system (i.e. not for GPS coordinates). 
# 
# Resources: 
# - https://www.alanzucconi.com/2017/03/13/positioning-and-trilateration/ 
# 
# Notes: 
# - This scipt is for 2D Cartesian coordinates only (as of now). 
# - To use this script, you must create a data set and call the 'trilateration' 
#   function with the created data. 
# - How to create a data set: 
#   - The number of distances provided must match the number of reference points. 
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
from enum import Enum 

#================================================================================


#================================================================================
# Constants 

MIN_REF_POINTS = 3   # Minimim number of reference points 
DIS_TOLERANCE = 5    # Distance to reference point tolerance (human error) 

class ErrorType(Enum): 
    NONE = 0             # No error 
    REF_LOW = 1          # Not enough reference points 
    REF_SAME = 2         # Multiple reference points are the same 
    REF_LINE = 3         # Reference points form a straight line 
    NO_SOLUTION = 4      # Provided distances don't produce a solution 

#================================================================================


#================================================================================
# Data 

# This data gets populated later 
num_ref_points = 0   # Number of reference points 
num_points = 0       # Number of points of interest 
Pn = []              # List of locations for points of interest 

#================================================================================


#================================================================================
# Configuration 

# Plot colour info 
c1 = '#ff7f0e' 
c2 = '#17becf' 

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
    return mse / len(locations) 

#================================================================================


#================================================================================
# Checks 

# Check user data 
def data_check(P, d): 
    return coordinate_check(P), distance_check(P, d) 


# Coordinate check 
def coordinate_check(P): 
    i = 0 
    same_coordinates = 0 
    straight_line_x = 1 
    straight_line_y = 1 

    # Check that there are a minimum number of reference points. 
    if (num_ref_points < MIN_REF_POINTS): 
        print("Not enough reference points.") 
        return ErrorType.REF_LOW 

    # Check that no two reference points are the same and that the reference 
    # points don't form a straight line. Multiple of the same reference point 
    # is redundant and can also lead to insufficient information if there are a 
    # total of less than 3 unique reference points. If reference points form a 
    # straight line then there will be two possible solutions which is not 
    # acceptable. We must check each reference point against all other reference 
    # points to verify there are no repeated points. A straight line can be 
    # checked but successively comparing one reference point to the next because 
    # all x or all y values must be the same to forma a line. 
    while (i < (num_ref_points - 1)): 
        # Same reference point check 
        j = i + 1 
        while (j < num_ref_points): 
            if ((P[i][0] == P[j][0]) and (P[i][1] == P[j][1])): 
                same_coordinates = 1 
            j += 1 
        
        # Straight line check 
        if (P[i][0] != P[i+1][0]): 
            straight_line_x = 0 
        if (P[i][1] != P[i+1][1]): 
            straight_line_y = 0 

        i += 1 
    
    # Same reference point error condition 
    if (same_coordinates): 
        print("Reference coordinates cannot be the same.") 
        return ErrorType.REF_SAME 

    # Straight line of reference points error condition 
    if (straight_line_x or straight_line_y): 
        print("Reference coordinates cannot form a straight line.") 
        return ErrorType.REF_LINE 
    
    return ErrorType.NONE 


# Distance check 
def distance_check(P, d): 
    dxx = [] 
    ref_index = 0 
    i = 0 

    # Find the distance between each reference point 
    while (ref_index < (num_ref_points - 1)): 
        j = ref_index + 1 
        while (j < num_ref_points): 
            dis = planer_distance(P[ref_index][0], P[ref_index][1], P[j][0], P[j][1])
            dxx.append(dis) 
            j += 1 
        ref_index += 1 

    # Check that all points of interest have distances that can produce a valid 
    # solution. This loops through each point of interest (d list) and checks 
    # the distances of each point of interest to each reference point. 
    while (i < num_points): 
        ref_index = 0 
        j = 0 
        while (j < (num_ref_points - 1)): 
            k = j + 1 
            while (k < num_ref_points): 
                if (dxx[ref_index] > (d[i][j] + d[i][k] + DIS_TOLERANCE)): 
                    print("Distance to reference points is less than distance" \
                           "between reference points.") 
                    return ErrorType.NO_SOLUTION 
                ref_index += 1 
                k += 1 
            j += 1 
        i += 1 

    return ErrorType.NONE 

#================================================================================


#================================================================================
# Application 

# User function 
def trilateration(P, d): 
    global num_ref_points 
    global num_points 
    num_ref_points = len(P) 
    num_points = len(d) 

    # Make sure the user provided data is valid before proceeding 
    error1, error2 = data_check(P, d) 
    if ((error1 != ErrorType.NONE) or (error2 != ErrorType.NONE)): 
        sys.exit() 

    # Find the approximate location of each point of interest 
    for D in d: 
        # Update the calculation location 
        result = minimize(
            mse,                                         # The error function 
            D[num_ref_points],                           # The initial guess 
            args=(P, D),                                 # Additional parameters for mse 
            method='L-BFGS-B',                           # The optimisation algorithm 
            options={ 'ftol': 1e-5, 'maxiter': 1e+7 })   # Tolerance & max iterations 

        Pn.append([result.x, D[-1]]) 


    # Set up plot 
    fig, ax = plt.subplots() 

    # Plot the reference points (landmarks) 
    for p in P: 
        ax.scatter(p[0], p[1], s=10, c=c1) 
        ax.annotate(p[2], (p[0], p[1])) 

    # Plot the approximate points 
    for p in Pn: 
        ax.scatter(p[0][0], p[0][1], s=20, c=c2) 
        ax.annotate(p[1], (p[0][0], p[0][1])) 

    # Format the show the plot 
    ax.set_title("Subnautica") 
    ax.set_xlabel("Latitude (m)") 
    ax.set_ylabel("Longitude (m)") 
    ax.set_xlim(-2000, 2000) 
    ax.set_ylim(-2000, 2000) 
    plt.show() 

#================================================================================
