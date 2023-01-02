#================================================================================
# File: waypoint_mission.py 
# 
# Author: Sam Donnelly (samueldonnelly11@gmail.com)
# 
# Description: Waypoint mission planner 
# 
# Date: December 20, 2022 
#================================================================================

#================================================================================
# TODO 
# - Use this script to extract GPS coordinates from online (user defined) and 
#   format them into a file for sending to the robot. 
# - Plot a full mission onto a map? This may be redundant as I probably won't 
#   write coordinates down without looking at the map first.  
#================================================================================


#================================================================================
# Steps 
#================================================================================


#================================================================================
# Notes 
# - The current string comparision method for inputs doesn't acount for spaces 
#   after word during input  
#================================================================================


#================================================================================
# Includes 

# Paths 
import file_locations 

# Libraries 
import atexit 
import sys 

# Add include support for other sub-directories 
# TODO add error handling for the directory check 
sys.path.append(file_locations.repo_folder + "simulation/library/") 

# Files 
import geoplaner 
from user_interface import user_input 

#================================================================================


#================================================================================
# Variables 

state = 0   # State tracker 

# Prompts 
operation = "Operation: " 
app = "App: "

# Program termination 
terminate = "exit"                  # Exit program string input 

#================================================================================


#================================================================================
# Error handling functions 

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
# State functions 

#
# brief: 
# 
# description: 
#
def op_select(): 
    print("\nOperations: ") 
    print("1. Get") 
    print("2. Send\n") 


#
# brief: 
# 
# description: 
#
def app_select(): 
    print("\nApp tp use: ") 
    print("1. geoplaner") 
    print("2. Other") 
    print("3. Back\n") 


#
# brief: 
# 
# description: 
#
def s0_op_select(prompt): 
    valid, cmd = user_input(prompt, terminate, str) 

    if ((not valid) and (cmd == -1)): 
        exit() 

    cmd = cmd.lower() 

    if (valid): 
        if ((cmd == "get") or (cmd == "1")): 
            return True 
        else: 
            print("\nNot a valid operation.") 
    
    return False 


#
# brief: 
# 
# description: 
#
def s1_get_select(prompt): 
    valid, cmd = user_input(prompt, terminate, str) 

    if ((not valid) and (cmd == -1)): 
        exit() 

    cmd = cmd.lower() 

    if (valid): 
        if ((cmd == "geoplaner") or (cmd == "1")): 
            geoplaner.geo_planer() 
            return True 
        elif ((cmd == "back") or (cmd == "3")): 
            return True 
        else: 
            print("\nNot a valid app.") 
    
    return False 

#================================================================================


#================================================================================
# Main code 

#==================================================
# Setup 

# Configure the exit handler 
atexit.register(exit_handler) 

# Define the file save directory 

#==================================================

# Main loop 
while (True): 
    #==================================================
    # State machine 

    # Operation select UI state 
    if (state == 0): 
        op_select() 
        state = 1 
    
    # Operation select state 
    elif (state == 1): 
        if (s0_op_select(operation)): 
            state = 2 

    # Getter application select UI state 
    elif (state == 2): 
        app_select() 
        state = 3 
    
    # Getter application select state 
    elif (state == 3): 
        if (s1_get_select(app)): 
            state = 0 
    
    # Send coordinates state 
    elif (state == 4): 
        state = 0 

    # Default back to operation select state 
    else: 
        state = 0 
    
    #==================================================

#================================================================================
