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
        return 

    # Get the current directory 

    #==================================================

    #==================================================
    # Search the file needed 

    # Check for gpx files 
    gpx_files = glob.glob(geoplaner_file_path + "*.gpx") 

    for file in gpx_files: 
        print(file) 

    # Check the time stamp on the files 

    #==================================================

#================================================================================
