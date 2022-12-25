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
radius = 1                          # Define the radius of the Earth 

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
def user_input(prompt, check, data_type): 
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
# brief: Generates the equation of a plane 
# 
# description: 
#
def plane_generation(): 
    print("Plane generation") 
    
    
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
#              The equation used to calculate heading angle produces a number from 
#              -90 degrees to 90 degrees. Given a compass heading can be anywhere 
#              from 0 to 360 degrees, or -180 to 180 degrees depending on the reference 
#              point is established, this means there are two possible solutions for 
#              a single calculation (each solution shifted 180 degrees from the other). 
#              To distinguish which solution to use, we look at the signs on the 
#              numerator and denominator of the angle equation to determine what 
#              quadrant the solution lies in. Using this information we can adjust the 
#              angle to give a unique solution. 
#
def compass_heading(lat1, lon1, lat2, lon2): 
    # Convert angles to radians 
    lat1 = deg_to_rad(lat1) 
    lon1 = deg_to_rad(lon1) 
    lat2 = deg_to_rad(lat2) 
    lon2 = deg_to_rad(lon2) 
    
    # Calculate the components of the initial course equation 
    eq_num = np.cos(lat2)*np.sin(lon2-lon1) 
    eq_den = np.cos(lat1)*np.sin(lat2) - np.sin(lat1)*np.cos(lat2)*np.cos(lon2-lon1) 
    
    # Calculate the initial heading angle 
    init_heading = np.arctan(eq_num/eq_den) 
    
    #==================================================
    # # Adjust the heading angle to get a unique solution (see function description) 

    # -180 to 180 degree logic 
    if (eq_den < 0): 
        if (eq_num >= 0): 
            init_heading = init_heading + np.pi 
        else: 
            init_heading = init_heading - np.pi 

    # 0-360 degree logic 

    #==================================================
    
    return init_heading 


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
def great_circle_path(r1, r2, gps1, gps2):
    # Pass the calculated xyz coordinates to this function 
    
    # Use the coordinates as your two plane vectors (origin reference) and find 
    # the cross product between them to get the normal vector of the plane 
    n = np.cross(r1, r2) 
    m = [0, 0, 1]  # xy plane normal vector 
    
    # We can choose to plot the entire great circle path (maybe save this for the 
    # heading variations script) or plot just the portion between the two points. 
    # We will do the latter. 
    
    # Take *all the x and y coordinates for the Earth plot (pass as an argument 
    # here) and use that with the normal vector to calculate all the z points of 
    # the great circle. 
    
    # *To get just the path between the two points, use the initial and final x 
    # and y coordinates as bounds on the full data set. Generate a new set of 
    # of 1D xyz coordinates (not 2D used to plot the Earth surface), plot that 
    # and that should give you the path. 
    
    # Within this code the path calculation only serves as a representation. It 
    # will serve in calculating heading variations along the great circle later. 
    
    # Calculate the angle between the normal vectors 
    cos_theta = abs(np.dot(m, n) / (np.linalg.norm(m) * np.linalg.norm(n))) 
    
    print("\ncos_theta: ") 
    print(cos_theta) 
    
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
    
    # Generate circle points in the xy plane 
    num_points = 100 
    angle = np.linspace(0, 2*np.pi, num_points) 
    
    x_rot = [0] * num_points 
    y_rot = [0] * num_points 
    z_rot = [0] * num_points 
    
    for i in range(num_points): 
        vector = np.dot([radius*np.cos(angle[i]), radius*np.sin(angle[i]), 0], R) 
        x_rot[i] = vector[0] 
        y_rot[i] = vector[1] 
        z_rot[i] = vector[2] 
        
    return x_rot, y_rot, z_rot 

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
    r1 = [x1, y1, z1] 
    r2 = [x2, y2, z2] 
    gps1 = [lat1, lon1] 
    gps2 = [lat2, lon2] 
    x_rot, y_rot, z_rot = great_circle_path(r1, r2, gps1, gps2) 

    #==================================================

    #==================================================
    # Set up the figure to show data 

    # Calling this in the loop is required to repeatedly show a plot 

    # Create the figure 
    fig = plt.figure() 
    ax = fig.add_subplot(projection='3d')

    # Plot the Earth data 
    ax.plot_surface(X, Y, Z) 
    
    # Plot the coordinates 
    ax.scatter(x1, y1, z1, marker="v", c=0.5) 
    ax.scatter(x2, y2, z2, marker="o", c=0.4) 
    
    # Plot the great circle 
    ax.scatter(x_rot, y_rot, z_rot) 

    # Format the plot 
    plt.xlabel('x')
    plt.ylabel('y') 
    # ax.set_aspect('equal') 
    ax.set_aspect('auto') 
    
    # Show the plot 
    plt.show() 

    # Close the plot 
    plt.close() 

    #==================================================

#==================================================

#================================================================================
