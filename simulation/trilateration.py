#================================================================================
# File: trilateration.py 
# 
# Author: Sam Donnelly (samueldonnelly11@gmail.com)
# 
# Description: Find location of a point given the points distances to 3 other 
#              known locations. 
#
# Date: November 12, 2024 
#================================================================================


#================================================================================
# Imports 

import sys 

#================================================================================


#================================================================================
# Reference points 

# Coordinate 1 
x1 = 0 
y1 = 0 

# Coordinate 2 
x2 = 0 
y2 = 1 

# Coordinate 3 
x3 = 1 
y3 = 2 

# New coordinate 
xn = 0 
yn = 0 
r1n = 0 
r2n = 0 
r3n = 0 

#================================================================================


#================================================================================
# Classes 

# class Location: 
#     def __init__(self, r1n, r2n, r3n): 
#         self.r1n = r1n 
#         self.r2n = r2n 
#         self.r3n = r3n 
#         self.xn 
#         self.yn 

#================================================================================


#================================================================================
# Calculations 

# Square a number 
def sqr(x): 
    return x**2 

# Constant 12 
def c12_calc(): 
    num = sqr(r1n) + sqr(r2n) + sqr(x2) - sqr(x1) + sqr(y2) - sqr(y1) 
    den = 2 * (x2 - x1) 
    return num / den 

# Constant 13 
def c13_calc(): 
    num = sqr(r1n) + sqr(r3n) + sqr(x3) - sqr(x1) + sqr(y3) - sqr(y1) 
    den = 2 * (y3 - y1) 
    return num / den 

# Xn coordinate 
def xn_calc(): 
    num = c12_calc() - c13_calc() * (y2 - y1) / (x2 - x1) 
    den = 1 - (x3 - x1) * (y2 - y1) / ((y3 - y1) * (x2 - x1)) 
    return num / den 

# Yn coordinate 
def yn_calc(): 
    return c13_calc() - xn_calc() * (x3 - x1) / (y3 - y1) 

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
def distance_check(): 
    print("bad!") 

#================================================================================


#================================================================================
# Application 

# Make sure the reference points are valid 
if (coordinate_check()): 
    sys.exit() 

print("yooooyo") 

#================================================================================
