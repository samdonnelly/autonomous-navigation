#================================================================================
# File: trajectory_calculation.py 
# 
# Author: Sam Donnelly (samueldonnelly11@gmail.com)
# 
# Description: Trajectory calculation and plotter 
# 
# Date: December 20, 2022 
#================================================================================

#================================================================================
# Steps 
# 1. Create a 3D plot space 
# 2. Create a 3D surface within the plot space (sphere and/or section of sphere) 
# 3. Make user input for inputting coordinates 
# 4. Plot the input coordinates 
# 5. Calculate the path (heading and arc) between the points and plot on the surface 
#================================================================================


#================================================================================
# Notes 
# - Could make a math file for easier insertion of constants and such 
# - The ranges on the sphere linspaces can be used to plot only a portion of the sphere 
# - 'outer' creates two dimensions from 1D data (u & v). The calculations done to get 
#   the xyz coordinates are the xyz coordinate equations of a sphere. 
# - 
#================================================================================


#================================================================================
# TODO 
# - 
# - Scatter points (coordinates) can only be seen on the surface plot at some angles 
# - User input verification 
# - Clear old plot points 
# - Update plot without closing the plot 
# - Convert a user defined geofence into linspace ranges and plot that portion of the 
#   sphere 
# - 
#================================================================================


#================================================================================
# Includes 

import matplotlib.pyplot as plt 
import numpy as np 

#================================================================================


#================================================================================
# Variables 

# Define the radius of the Earth 
radius = 1

#================================================================================


#================================================================================
# Main 

#==================================================
# Spherical Plot Setup 

# Create the figure 
fig = plt.figure() 
ax = fig.add_subplot(projection='3d') 

# Generate base data for the figure 
u = np.linspace(0, 2*np.pi, 100) 
v = np.linspace(0, np.pi, 100) 
x = 0.95*np.outer(np.cos(u), np.sin(v)) 
y = 0.95*np.outer(np.sin(u), np.sin(v)) 
z = 0.95*np.outer(np.ones(np.size(u)), np.cos(v)) 

# # Plot the base data 
ax.plot_surface(x, y, z) 

# Configure the figure 
plt.xlabel('x')
plt.ylabel('y')

# Format the plot 
ax.set_aspect('equal') 

#==================================================

#==================================================
# Main loop 

while (True): 
    #==================================================
    # Get user input 

    # Get the start coordinate 
    print("\nStart point:") 
    lat = input("Latitude: ") 
    lon = input("Longitude: ") 
    coordinate1 = "Start point: " + lat + ", " + lon 

    # Calculate the location on the sphere 
    lat1 = (90.0 - float(lat)) * np.pi / 180.0 
    lon1 = ((float(lon) + 90.0) + 90.0) * np.pi / 180.0 
    x1 = radius*np.cos(lon1)*np.sin(lat1) 
    y1 = radius*np.sin(lon1)*np.sin(lat1) 
    z1 = radius*np.cos(lat1) 

    # Get the end coordinate 
    print("\nEnd point:") 
    lat = input("Latitude: ") 
    lon = input("Longitude: ") 
    coordinate2 = "End point: " + lat + ", " + lon 

    # Calculate the location onn the sphere 
    lat2 = (90.0 - float(lat)) * np.pi / 180.0 
    lon2 = ((float(lon) + 90.0) + 90.0) * np.pi / 180.0 
    x2 = radius*np.cos(lon2)*np.sin(lat2) 
    y2 = radius*np.sin(lon2)*np.sin(lat2) 
    z2 = radius*np.cos(lat2) 

    # Plot the coordinates 
    ax.scatter(x1, y1, z1, marker="v", c=0.5) 
    ax.scatter(x2, y2, z2, marker="o", c=0.4) 

    # Print the coordinates 
    print("\n" + coordinate1 + "\n" + coordinate2) 

    #==================================================
    
    # Show the plot 
    plt.show() 

#==================================================

#================================================================================
