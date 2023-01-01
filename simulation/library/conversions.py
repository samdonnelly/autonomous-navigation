#================================================================================
# File: 
# 
# Author: 
# 
# Description: 
# 
# Date: 
#================================================================================

#================================================================================
# Notes 
#================================================================================


#================================================================================
# Includes 

import numpy as np 

#================================================================================


#================================================================================
# Functions 

#
# brief: Converts degrees to radians 
# 
# @param angle : angle in degrees 
# @return 1 : angle in radians 
#
def deg_to_rad(angle): 
    return (angle * np.pi / 180.0) 


#
# brief: Converts radians to degrees 
# 
# @param angle : angle in radians 
# @return 1 : angle in degrees 
#
def rad_to_deg(angle): 
    return (angle * 180.0 / np.pi) 

#================================================================================
