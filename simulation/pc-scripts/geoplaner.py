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
import atexit 

# Scripts 
import file_locations 

#================================================================================


#================================================================================
# Error handling functions 

#
# brief: 
# 
# description: 
#
def invalid_file(): 
    print("\nFile or directory could not be found") 
    exit() 

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
# Setup 

# Configure the exit handler 
atexit.register(exit_handler) 

#================================================================================


#================================================================================
# Directories 

# Check for a valid Downloads folder 
if (os.path.exists(file_locations.downloads)): 
    path = file_locations.downloads 
else: 
    invalid_file() 

print("file location good") 

# Get the current directory 

#================================================================================
