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
# Steps 
# 1. Search for a gpx file in the downloads folder. This is the default location 
#    that files are saved to from geoplaner. Have the script search by file type 
#     
#================================================================================


#================================================================================
# Notes 
# - Do we want to save old coordinate data?  
#================================================================================


#================================================================================
# Includes 

# Libraries 
import os 
import glob 
import time 

# Scripts 
import file_locations 

#================================================================================


#================================================================================
# User functions 

def geo_planer(): 
    #==================================================
    # Get and check directories 

    # Check for a valid Downloads folder 
    if (os.path.exists(file_locations.downloads)): 
        geoplaner_file_path = file_locations.downloads 
    else: 
        print("\nInvalid path to look for gpx files.") 
        return 

    # Get the current directory 

    #==================================================

    #==================================================
    # Search the file needed 

    # TODO create a files file in the library for the following 

    # Check for gpx files 
    gpx_files = glob.glob(geoplaner_file_path + "*.gpx") 

    if (gpx_files == []): 
        print("No gpx files.") 
        return 
    
    # Check the time stamp on the files to get the most recent file 
    for file in gpx_files: 
        print("") 
        print(file) 
        mod_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(os.path.getmtime(file))) 
        print("Last modified time: ", mod_time) 
        print((type(mod_time))) 

    #==================================================

#================================================================================
