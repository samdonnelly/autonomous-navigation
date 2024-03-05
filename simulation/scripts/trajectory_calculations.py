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
# - Make a script that interfaces with Google maps to record (and plot?) coordinates  
#================================================================================


#================================================================================
# TODO 
# - Add a conversion between the different coordinate formats (here or in another file?) 
# - Scatter points (coordinates) can only be seen on the surface plot at some angles 
# - Clear old plot points (or keep them?) 
# - Update plot without closing the plot 
# - Convert a user defined geofence into linspace ranges and plot that portion of the 
#   sphere. The input coordinates would then have to be checked against this. 
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
# Global variables 

# Conditional compilation 
HEADING_360 = 1                     # 360 degree heading mode - otherwise +/-180

# Constants 
radius = 1                          # Radius used for coordinate calculation 
earth_radius = 6371.0               # Average radius of the Earth (km) 

# Pre-defined strings 
lat_prompt = "Latitude: "
lon_prompt = "Longitude: "
terminate = "exit"                  # Exit program string input 

#================================================================================


#================================================================================
# General Purpose Functions 

#
# brief: Checks user input for a valid entry 
# 
# description: The function prints a prompt (specified in the arguments) to the 
#              console and takes the user input. This input is first checked 
#              against the 'check' argument, which if matched, will then 
#              terminate the script. If there is no match then the 'data_type' 
#              argument is used to determine how to interpret the user input. 
#              If the user input aligns with the data types specified then 
#              'True' is returned along with the user input formatted 
#              according to the data type. If the input doesn't match the 
#              data types then 'False' is returned and the user input returned 
#              is irrelevant.  
# 
# @param prompt : string printed to console to prompt the user as needed 
# @param check : string to check user input against for terminating the program 
# @param data_type : expected data type input from the user 
# @return 1 : True if user input matches expected format, False otherwise 
# @return 2 : contents of the user input formatted as needed if it's a valid input 
#
def user_input(
    prompt, 
    check, 
    data_type): 

    # Get the user input 
    command = input(prompt) 
        
    # Check for exit 
    # String comparision 
    for i in range(len(check)): 
        try: 
            if (command[i].lower() != check[i]):
                break 
        except IndexError: 
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
        
        
#
# brief: Code called upon termination of the script 
# 
# description: Code that should be run before the script terminates should be put here. 
#              This code will only be run once the program has begun the termination 
#              process. 
#
def exit_handler(): 
    print("\nProgram terminated.\n") 

#================================================================================


#================================================================================
# Program Specific Functions 

#
# brief: Calculate compass heading (degrees) between current and destination location 
# 
# description: Uses the great circle navigation equation for initial heading to 
#              get a compass heading. The heading is the angle from true North. 
#              
#              This calculated heading can be used to compare to a magnetometer 
#              heading to know which direction to travel. 
#              
#              This equation comes from spherical triginometry. 
#              
#              Requires np.seterr(divide='ignore') in the code somewhere to suppress 
#              RuntimeWarnings for zero division. 
#              
#              The equation used to calculate heading angle produces a number from 
#              -90 degrees to 90 degrees. Given a compass heading can be anywhere 
#              from 0 to 360 degrees, or -180 to 180 degrees depending on the reference 
#              point established, which means there are two possible solutions for 
#              a single calculation (each solution shifted 180 degrees from the other). 
#              To distinguish which solution to use, we look at the signs on the 
#              numerator and denominator of the angle equation to determine what 
#              quadrant the solution lies in. Using this information we can adjust the 
#              angle to give a unique solution. 
# 
# @param lat1 : coordinate 1 (starting/current location) latitude 
# @param lon1 : coordinate 1 (starting/current location) longitude 
# @param lat2 : coordinate 2 (final/destination location) latitude 
# @param lon2 : coordinate 2 (final/destination location) longitude 
# @return init_heading : initial heading (degrees) between coordinates 
#
def compass_heading(
    lat1, 
    lon1, 
    lat2, 
    lon2): 
    
    # Convert angles to radians 
    lat1 = deg_to_rad(lat1) 
    lon1 = deg_to_rad(lon1) 
    lat2 = deg_to_rad(lat2) 
    lon2 = deg_to_rad(lon2) 
    
    # Calculate the numerator and denominator of the initial heading equation 
    eq_num = np.cos(lat2)*np.sin(lon2-lon1) 
    eq_den = np.cos(lat1)*np.sin(lat2) - np.sin(lat1)*np.cos(lat2)*np.cos(lon2-lon1) 
    
    # Calculate the initial heading 
    init_heading = np.arctan(eq_num/eq_den) 
    
    #==================================================
    # Adjust the heading angle to get a unique solution (see function description) 

    # Below are the signs of the numerator and denominator for each quadrant 
    # Q1 (top right):    num: (+), den: (+) 
    # Q2 (bottom right): num: (+), den: (-) 
    # Q3 (bottom left):  num: (-), den: (-) 
    # Q4 (top left):     num: (-), den: (+) 

    if HEADING_360: 
        # 0-360 degree logic 
        if (eq_den < 0): 
            init_heading += np.pi
        elif (eq_num < 0): 
            init_heading += 2*np.pi
    else: 
        # +/-180 degree heading logic 
        if (eq_den < 0): 
            if (eq_num < 0): 
                init_heading -= np.pi 
            else: 
                init_heading += np.pi 

    #==================================================
    
    return init_heading 


#
# brief: Coordinate location on sphere calculation 
# 
# description: 
# 
# @param lat : 
# @param lon : 
# @return lat : 
# @return lon : 
# @return x : 
# @return y : 
# @return z : 
#
def xyz_gps_coordinate(
    lat, 
    lon): 
    
    # Adjust the coordinates - adjusted based on matplotlib plot orientation 
    lat = deg_to_rad(90.0 - lat) 
    lon = deg_to_rad(lon) 
    
    # Get the XYZ points of the coordinates 
    x = radius*np.cos(lon)*np.sin(lat) 
    y = radius*np.sin(lon)*np.sin(lat) 
    z = radius*np.cos(lat) 
    
    return lat, lon, x, y, z


#
# brief: Calculate the great circle path 
# 
# description: Generates the great circle points between the two user input 
#              coordinates so that it can be plotted. 
#  
#              Steps: 
#              1. Find the plane that the two input coordinates are in by taking 
#                 the cross product between the coordinate vectors with reference 
#                 to the origin. 
#              2. Create a rotation matrix between the xy plane and the great circle 
#                 plane. 
#              3. Set the bounds of the great circle to in between the two input 
#                 coordinates so that only that part of the great circle is plotted. 
#                 Use these bounds to calculate all the points to plot.  
#              4. Translate the path points into the great circle plane and return 
#                 the result so it can be plotted.  
# 
#              Note: We can plot the entire great circle plane by defining 'angle' 
#                    from 0 to 2*pi and translating all of those points into the 
#                    great circle plane: 
#                    angle = np.linspace(0, 2*np.pi, num_points) 
# 
#              Note: Within this code the path calculation only serves as a 
#                    representation. It will serve in calculating heading variations 
#                    along the great circle later. 
# 
# @param r1 : 
# @param r2 : 
#
def great_circle_path(
    r1, 
    r2):
    
    # Local variables 
    num_points = 100       # Number of great circle plot points 
    angle = []             # Empty array to hold great circle coordinate angles 
    x_rot = []             # Empty array to hold x coordinates of the great circle 
    y_rot = []             # Empty array to hold y coordinates of the great circle 
    z_rot = []             # Empty array to hold z coordinates of the great circle 

    # Define the xy plane and great circle plane 
    m = [0, 0, 1]              # xy plane normal vector 
    n = np.cross(r1, r2)       # Great circle plane (of coordinates) normal vector 

    #==================================================
    # Define the translation from the xy plane to the great circle plane 

    # Calculate the angle between the normal vectors 
    cos_theta = -np.dot(m, n) / (np.linalg.norm(m) * np.linalg.norm(n)) 
    
    # Calculate the rotation axis 
    rot_axis = np.cross(m, n) / np.linalg.norm(np.cross(m, n)) 
    
    # Calculate the rotation matrix 
    s = np.sqrt(1 - cos_theta**2) 
    C = 1 - cos_theta 
    x = rot_axis[0] 
    y = rot_axis[1] 
    z = rot_axis[2] 
    R = [[x*x*C + cos_theta, x*y*C - z*s,       x*z*C + y*s], 
         [y*x*C + z*s,       y*y*C + cos_theta, y*z*C - x*s], 
         [z*x*C - y*s,       z*y*C + x*s,       z*z*C + cos_theta]] 

    # Calculate the inverse matrix 
    R_I = np.linalg.inv(R) 

    #==================================================
    
    #==================================================
    # Set the bounds of the great circle section 

    # Translate the input coordinate vectors to the xy plane 
    r1_xy = np.dot(r1, R_I) 
    r2_xy = np.dot(r2, R_I) 

    # Find the angle of the translated coordinates from the x axis 
    r1_xy_theta = np.arccos(r1_xy[0] / radius) 
    r2_xy_theta = np.arccos(r2_xy[0] / radius) 

    # Adjust the angle at account for the acceptable range of theta (-pi to pi) 
    if (r1_xy[1] < 0): 
        r1_xy_theta = -r1_xy_theta 
    
    if (r2_xy[1] < 0): 
        r2_xy_theta = -r2_xy_theta 

    # Determine the relative location of the translated coordinates 
    r_xy_theta_min = min(r1_xy_theta, r2_xy_theta) 
    r_xy_theta_max = max(r1_xy_theta, r2_xy_theta) 
    
    # Define equivalent great circle path points in the xy plane 
    if (abs(r2_xy_theta - r1_xy_theta) <= np.pi):  
        angle = np.linspace(r_xy_theta_min, r_xy_theta_max, num_points)
    
    else:   # Crosses pi/-pi boundary 
        diff = int(((np.pi - r_xy_theta_max) / \
                    ((np.pi - r_xy_theta_max) + (r_xy_theta_min + np.pi)))*num_points) 
        angle.extend(np.linspace(r_xy_theta_max, np.pi, diff)) 
        angle.extend(np.linspace(-np.pi, r_xy_theta_min, num_points - diff)) 
     
    #==================================================
    
    # Translate the great circle path points into the great circle plane 
    for i in range(num_points): 
        # Rotate to the correct plane 
        vector = np.dot([radius*np.cos(angle[i]), radius*np.sin(angle[i]), 0], R) 
        x_rot.append(vector[0]) 
        y_rot.append(vector[1]) 
        z_rot.append(vector[2]) 
    
    return x_rot, y_rot, z_rot 


#
# brief: Calculates the central angle between two points on the great circle 
# 
# description: 
#              The cos and sin inputs were readjusted to their original input from 
#              their converted matplotlib axis format so that the correct angle 
#              would be calculated. 
# 
# @param gps1 : 
# @param gps2 : 
#
def central_angle(
    gps1, 
    gps2): 
    
    # Calculate the parts of the central angle equation 
    eq1 = np.cos(np.pi/2 - gps2[0])*np.sin(gps2[1]-gps1[1]) 
    eq2 = np.cos(np.pi/2 - gps1[0])*np.sin(np.pi/2 - gps2[0]) 
    eq3 = np.sin(np.pi/2 - gps1[0])*np.cos(np.pi/2 - gps2[0])*np.cos(gps2[1]-gps1[1]) 
    eq4 = np.sin(np.pi/2 - gps1[0])*np.sin(np.pi/2 - gps2[0]) 
    eq5 = np.cos(np.pi/2 - gps1[0])*np.cos(np.pi/2 - gps2[0])*np.cos(gps2[1]-gps1[1]) 
    
    # Calculate and return the central angle 
    return np.arctan2(np.sqrt((eq2-eq3)**2 + eq1**2), (eq4 + eq5)) 

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
# Spherical plot data generation 

# Generate base data for the figure (Earth) 
u = np.linspace(0, 2*np.pi, 100) 
v = np.linspace(0, np.pi, 100) 
X = (radius*0.95)*np.outer(np.cos(u), np.sin(v)) 
Y = (radius*0.95)*np.outer(np.sin(u), np.sin(v)) 
Z = (radius*0.95)*np.outer(np.ones(np.size(u)), np.cos(v)) 

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

    #==================================================

    #==================================================
    # Determine location and direction 
        
    # Calculate the heading at the starting point 
    initial_heading = compass_heading(lat1, lon1, lat2, lon2) 
    print("\nInitial heading: " + str(rad_to_deg(initial_heading))) 
        
    # Calculate the coordinate location on the sphere 
    lat1, lon1, x1, y1, z1 = xyz_gps_coordinate(lat1, lon1) 
    lat2, lon2, x2, y2, z2 = xyz_gps_coordinate(lat2, lon2) 
    
    # Calculate the great circle path  
    x_rot, y_rot, z_rot = great_circle_path([x1, y1, z1], [x2, y2, z2]) 

    #==================================================

    #==================================================
    # Calculate the distance to travel 

    cen_angle = central_angle([lat1, lon1], [lat2, lon2]) 
    distance = cen_angle*earth_radius 

    print("Surface distance between coordinates: " + str(distance) + " km") 

    #==================================================

    #==================================================
    # Set up the figure to show data 

    # Calling this in the loop is required to repeatedly show a plot 

    # Create the figure 
    fig = plt.figure() 
    ax = fig.add_subplot(projection='3d')

    # Plot the Earth data 
    # ax.plot_surface(X, Y, Z) 
    
    # Plot the coordinates 
    ax.scatter(x1, y1, z1, marker="v", c=0.5) 
    ax.scatter(x2, y2, z2, marker="o", c=0.4) 
    
    # Plot the great circle 
    ax.plot(x_rot, y_rot, z_rot) 

    # Format the plot 
    plt.xlabel('x')
    plt.ylabel('y') 
    ax.set_xlim(-1.1, 1.1) 
    ax.set_ylim(-1.1, 1.1) 
    ax.set_zlim(-1.1, 1.1) 
    
    # Show the plot 
    plt.show() 

    # Close the plot 
    plt.close() 

    #==================================================

#==================================================

#================================================================================
