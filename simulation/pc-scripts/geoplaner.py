#================================================================================
# File: geoplaner.py 
# 
# Author: Sam Donnelly (samueldonnelly11@gmail.com) 
# 
# Description: Handles the gpx files containing GPS coordinates from geoplaner.com 
# 
# Date: December 31, 2022 
#================================================================================

#================================================================================
# TODO 
# - See if geoplaner can go beyond 99 points  
#================================================================================


#================================================================================
# Notes 
# - You can download routes from geoplaner and it saves them in the Downloads 
#   folder.  
#================================================================================


#================================================================================
# Includes 

# Libraries 
import os 
import glob 
import time 
from datetime import datetime as dt 
import re 

# Files 
import file_locations 

#================================================================================


#================================================================================
# Variables 

# gpx file definition 
gpx_start_line = 4                     # First line of data in gpx files 
gpx_index_jump = 3                     # Lines between coordinates 
gpx_end_file = "</rte>\n"              # Indicates the end of the gpx file 
gpx_num_regex = re.compile(r"\d\d")    # Coordinate number format 
gpx_lat = 'lat="'                      # Text preceeding the latitude 
gpx_lon = '" lon="'                    # Text preceeding the longitude 
gpx_end_line = '">\n'                  # Text at the end of a coordinate line 

#================================================================================


#================================================================================
# User functions 

def geo_planer(): 
    #==================================================
    # Get and check directories 

    # Get the read and save directories for files 
    if (os.path.exists(file_locations.downloads) and 
        os.path.exists(file_locations.gps_folder)): 
        read_path = file_locations.downloads 
        write_path = file_locations.gps_folder 
    else: 
        print("\nInvalid path to either read gpx files or write coordinate data.") 
        return 

    #==================================================

    #==================================================
    # Search for the needed file 

    # Local variables 
    gpx_files = [[], []] 

    # Check for gpx files 
    gpx_files[0] = glob.glob(read_path + "*.gpx") 

    if (gpx_files[0] == []): 
        print("\nNo gpx files.") 
        return 

    # Get the modification time stamp on each gpx file 
    for file in gpx_files[0]: 
        gpx_files[1].append(time.strftime('%y-%m-%d %H:%M:%S', \
            time.localtime(os.path.getmtime(file)))) 

    # Get the most recent gpx file 
    gpx_recent = gpx_files[0][0] 

    for i in range(len(gpx_files[0])-1): 
        # Get the modification time stamp on the current and next file 
        ts1 = dt.strptime(gpx_files[1][i], "%y-%m-%d %H:%M:%S") 
        ts2 = dt.strptime(gpx_files[1][i+1], "%y-%m-%d %H:%M:%S") 
        
        # Check for the most recent file and update as needed 
        if (ts2 > ts1): 
            gpx_recent = gpx_files[0][i+1] 

    #==================================================

    #==================================================
    # Read the file for coordinates and write to a new file 

    # Open the file to read 
    gpx_file = open(gpx_recent) 
    gpx_file_content = gpx_file.readlines() 

    # Get the file start index 
    gpx_index = gpx_start_line 

    # Read the contents of the file 
    while(True): 
        # Check for end of file 
        if (gpx_file_content[gpx_index] == gpx_end_file): 
            break 

        # Read the coordinate number (WP##-<letter>).
        gps_num = gpx_num_regex.search(gpx_file_content[gpx_index+1]).group() 

        # Read the latitude and longitude 
        gps_lat = (gpx_file_content[gpx_index])[ \
                   gpx_file_content[gpx_index].find(gpx_lat) + len(gpx_lat) : \
                   gpx_file_content[gpx_index].find(gpx_lon)] 
        
        gps_lon = (gpx_file_content[gpx_index])[ \
                   gpx_file_content[gpx_index].find(gpx_lon) + len(gpx_lon) : \
                   gpx_file_content[gpx_index].find(gpx_end_line)] 

        # Update the index 
        gpx_index += gpx_index_jump 

    # Write to the new file 
    # - Create a new file with an appropriate name (current date and index). 
    # - Check for the existance of the file name - if so modify the new name 
    # - Copy the coordinate number followed by the lat and lon into the file 
    #   each on a new line.  

    #==================================================

#================================================================================
