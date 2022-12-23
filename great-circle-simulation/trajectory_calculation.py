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
# - Scatter points (coordinates) can only be seen on the surface plot at some angles 
# - Clear old plot points (or keep them?) 
# - Update plot without closing the plot 
# - Convert a user defined geofence into linspace ranges and plot that portion of the 
#   sphere. The input coordinates would then have to be checked against this. 
# - Aspect ratio of plot on mac doesn't support 'equal' 
#================================================================================


#================================================================================
# Includes 

# Math and data libraries 
import matplotlib.pyplot as plt 
import numpy as np 

# Functional libraries 
import atexit

#================================================================================


#================================================================================
# Variables 

# Parameters 
radius = 1     # Define the radius of the Earth 

# Pre-defined strings 
lat_prompt = "Latitude: "
lon_prompt = "Longitude: "
terminate = "exit"             # Exit program string input 

#================================================================================


#================================================================================
# General Purpose Functions 

#
# brief: Checks user input for a valid entry 
# 
# description: 
#
def user_input(prompt, check, data_type): 
    # Get the user input 
    command = input(prompt) 
        
    # Check for exit 
    # String comparision 
    for i in range(len(check)): 
        if (command[i].lower() != check[i]):
            break 
    
    if (i == (len(check)-1)): 
        exit() 
    
    # Try converting the number and check for errors 
    if (data_type is float): 
        # Try converting to float and check for errors 
        try: 
            value = float(command) 
            return True, value
        except ValueError: 
            print("\nFloat conversion failed.\n") 
    
    elif (data_type is int): 
        # Try converting to int and check for errors 
        try: 
            value = int(float(command)) 
            return True, value
        except ValueError: 
            print("\nInteger conversion failed.\n") 
            
    else: 
        # Interpreted as a string 
        return True, command
            
    return False, 0


#
# brief: Generates the equation of a plane 
# 
# description: 
#
def plane_generation(): 
    print("Plane generation") 
    
    
#
# brief: Converts degrees to radians 
# 
# description: 
#
def deg_to_rad(angle): 
    return (angle * np.pi / 180.0) 


#
# brief: Converts radians to degrees 
# 
# description: 
#
def rad_to_deg(angle): 
    return (angle * 180.0 / np.pi) 
        
        
#
# brief: Code called upon termination of the script 
# 
# description: 
#
def exit_handler(): 
    print("\nProgram terminated.\n") 

#================================================================================


#================================================================================
# Program Specific Functions 

#
# brief: Calculate the heading at the starting point 
# 
# description: Uses the great circle navigation equation for initial heading to get 
#              a compass heading. This angle must be equated to the readings seen
#              from a magnetometer. The angle is the angle with reference to North or 
#              the longitude line. 
#              
#              This equation comes from spherical triginometry. 
#              
#              Requires np.seterr(divide='ignore') in the code somewhere to suppress 
#              RuntimeWarnings for zero division. 
#
def compass_heading(lat1, lon1, lat2, lon2): 
    # Convert angles to radians 
    lat1 = deg_to_rad(lat1) 
    lon1 = deg_to_rad(lon1) 
    lat2 = deg_to_rad(lat2) 
    lon2 = deg_to_rad(lon2) 
    
    # Calculate portions of final equation 
    eq1 = np.cos(lat2)*np.sin(lon2-lon1) 
    eq2 = np.cos(lat1)*np.sin(lat2) 
    eq3 = np.sin(lat1)*np.cos(lat2)*np.cos(lon2-lon1) 
    
    # Calculate the heading 
    heading = np.arctan(eq1/(eq2-eq3)) 
    
    # The calculation is accurate however there are two solutions for every 
    # calculation given the nature of arctan. The logic below is the only certainty 
    # I have in the angles. For all other cases it gets messy. 
    # See if you can use the final course to figure out how to compensate the 
    # initial course angle. 
    
    # If starting point is in the Northern hemisphere + equator (lat1 >= 0) 
    # - If lat2 >= lat1 then angle remains the same 
    
    # If starting point is in the Southern hemisphere (lat1 < 0) 
    # - If lat2 <= lat1: 
    #   - If (heading >= 0) then (angle - 180) 
    #   - If (heading < 0) then (angle + 180) 
    
    return heading 


#
# brief: Coordinate location on sphere calculation 
# 
# description: 
#
def xyz_gps_coordinate(lat, lon): 
    # Adjust the coordinates - adjusted based on matplotlib plot orientation 
    lat = (90.0 - lat) * np.pi / 180.0 
    lon = lon * np.pi / 180.0 
    
    # Get the XYZ points of the coordinates 
    x = radius*np.cos(lon)*np.sin(lat) 
    y = radius*np.sin(lon)*np.sin(lat) 
    z = radius*np.cos(lat) 
    
    return lat, lon, x, y, z


#
# brief: Calculate the great circle path 
# 
# description: 
#
def great_circle_path():
    print("Great circle path") 

#================================================================================


#================================================================================
# Main 

#==================================================
# Setup 

# Configure the exit handler 
atexit.register(exit_handler) 

# Suppress division RuntimeWarnings in numpy 
np.seterr(divide='ignore')  

#==================================================

#==================================================
# Spherical Plot Setup 

# Generate base data for the figure (Earth) 
u = np.linspace(0, 2*np.pi, 100) 
v = np.linspace(0, np.pi, 100) 
X = 0.95*np.outer(np.cos(u), np.sin(v)) 
Y = 0.95*np.outer(np.sin(u), np.sin(v)) 
Z = 0.95*np.outer(np.ones(np.size(u)), np.cos(v)) 

#==================================================

#==================================================
# Main loop 

while (True): 
    #==================================================
    # Get user input 

    # Get the start coordinate 
    print("\nStart point:") 
    while (True): 
        valid, lat1 = user_input(lat_prompt, terminate, float) 
        if (valid): break 
    while (True): 
        valid, lon1 = user_input(lon_prompt, terminate, float) 
        if (valid): break 

    # Get the end coordinate 
    print("\nEnd point:") 
    while (True): 
        valid, lat2 = user_input(lat_prompt, terminate, float) 
        if (valid): break 
    while (True): 
        valid, lon2 = user_input(lon_prompt, terminate, float) 
        if (valid): break 
        
    # Print the coordinates 
    print("\nCoordinates:") 
    print(str(lat1) + ", " + str(lon1)) 
    print(str(lat2) + ", " + str(lon2)) 
        
    # Calculate the heading at the starting point 
    heading = compass_heading(lat1, lon1, lat2, lon2) 
    print("\nHeading from start point: " + str(rad_to_deg(heading))) 
        
    # Calculate the coordinate location on the sphere 
    lat1, lon1, x1, y1, z1 = xyz_gps_coordinate(lat1, lon1) 
    lat2, lon2, x2, y2, z2 = xyz_gps_coordinate(lat2, lon2) 
    
    # Calculate the great circle path 

    #==================================================

    #==================================================
    # Set up the figure 

    # Calling this in the loop is required to repeatedly show a plot 

    # Create the figure 
    fig = plt.figure() 
    ax = fig.add_subplot(projection='3d')

    # Plot the Earth data 
    ax.plot_surface(X, Y, Z) 
    
    # Plot the coordinates 
    ax.scatter(x1, y1, z1, marker="v", c=0.5) 
    ax.scatter(x2, y2, z2, marker="o", c=0.4) 

    # Format the plot 
    plt.xlabel('x')
    plt.ylabel('y') 
    # ax.set_aspect('equal') 
    ax.set_aspect('auto') 

    #==================================================

    # # Close the plot 
    # plt.close() 
    
    # Show the plot 
    plt.show() 

#==================================================

#================================================================================
