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
# 4. Plot the input the points 
# 5. Calculate the path (heading and arc) between the points and plot on the surface 
#================================================================================


#================================================================================
# Includes 

import matplotlib.pyplot as plt 
import numpy as np 

#================================================================================


#================================================================================
# Variables 
#================================================================================


#================================================================================
# Main 

#==================================================
# Plot Setup 

# Create the figure 
fig = plt.figure() 
ax = fig.add_subplot(projection='3d') 

# Generate base data for the figure 
u = np.linspace(0, 2*np.pi, 100) 
v = np.linspace(0, np.pi, 100) 
x = 10*np.outer(np.cos(u), np.sin(v)) 
y = 10*np.outer(np.sin(u), np.sin(v)) 
z = 10*np.outer(np.ones(np.size(u)), np.cos(v)) 

# Plot the base data 
ax.plot_surface(x, y, z) 

# Format the plot 
# ax.set_aspect('equal') 

# Show the plot 
plt.show() 

#==================================================

#==================================================
# Main loop 

# while (True): 
#     # 
#     print("Hello") 

#==================================================

#================================================================================
