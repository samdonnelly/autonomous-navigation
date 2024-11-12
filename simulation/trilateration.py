#================================================================================
# File: 
# 
# Author: Sam Donnelly (samueldonnelly11@gmail.com)
# 
# Description: 
#
# Date: 
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
# Calculations 

# Square a number 
def sqr(x): 
    return x**2 

# C12 
def c12_calc(): 
    num = sqr(r1n) + sqr(r2n) + sqr(x2) - sqr(x1) + sqr(y2) - sqr(y1) 
    den = 2 * (x2 - x1) 
    return num / den 

# C13 
def c13_calc(): 
    num = sqr(r1n) + sqr(r3n) + sqr(x3) - sqr(x1) + sqr(y3) - sqr(y1) 
    den = 2 * (y3 - y1) 
    return num / den 

# Xn 
def xn_calc(): 
    num = c12_calc() - c13_calc() * (y2 - y1) / (x2 - x1) 
    den = 1 - (x3 - x1) * (y2 - y1) / ((y3 - y1) * (x2 - x1)) 
    return num / den 

# Yn 
def yn_calc(): 
    return c13_calc() - xn_calc() * (x3 - x1) / (y3 - y1) 

#================================================================================


#================================================================================
# Checks 

# Coordinate check 
def coordinate_check(): 
    error = 0 
    if ((x1 == x2 == x3) or (y1 == y2 == y3)): 
        print("Coordinates form a straight line") 
        error = 1 
    if (((x1 == x2) and (y1 == y2)) or 
        ((x1 == x3) and (y1 == y3)) or 
        ((x2 == x3) and (y2 == y3))): 
        print("Coordinates cannot be the same")
        error = 1 

#================================================================================


#================================================================================
# Application 

coordinate_check() 

#================================================================================
