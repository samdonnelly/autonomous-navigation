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
# - Make this whole thing better 
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
try: 
    sys.path.append(file_locations.repo_folder + "simulation/library/") 
except: 
    print("\nInvalid import file directory.") 
    exit() 

# Files 
import geoplaner 
from user_interface import user_input 

#================================================================================


#================================================================================
# Variables 

state = 0          # State tracker 
state_return = 0   # State return value 

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
# brief: State 0 - Operation select UI 
# 
# description: 
#
def s0_op_select_ui(): 
    print("\nOperations: ") 
    print("1. Get")              # Create a new coordinate set 
    print("2. Send")             # Send an existing coordinate set 
    print("3. Delete\n")         # Delete an existing coordinate set 


#
# brief: State 1 - Operation select 
# 
# description: 
#
def s1_op_select(prompt): 
    valid, cmd = user_input(prompt, terminate, str) 

    if ((not valid) and (cmd == -1)): 
        exit() 

    cmd = cmd.lower() 

    if (valid): 
        if ((cmd == "get") or (cmd == "1")): 
            return 1
        
        elif ((cmd == "send") or (cmd == "2")): 
            return 2 
        
        elif ((cmd == "delete") or (cmd == "3")): 
            return 3 
        
        else: 
            print("\nNot a valid operation.") 
    
    return False 


#
# brief: State 2 - Get coordinates UI 
# 
# description: 
#
def s2_app_select_ui(): 
    print("\nApp tp use: ") 
    print("1. geoplaner") 
    print("2. Other") 
    print("3. Back\n") 


#
# brief: State 3 - Get coordinates 
# 
# description: 
#
def s3_get_select(prompt): 
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


#
# brief: State 4 - Send coordinates UI 
# 
# description: 
#
def s4_send_select_ui(): 
    print("\nSending not yet implemented.")  


#
# brief: State 5 - Send coordinates 
# 
# description: 
#
def s5_send_select(): 
    return 


#
# brief: State 6 - Send coordinates UI 
# 
# description: 
#
def s6_delete_select_ui(): 
    print("\Deleting not yet implemented.")  


#
# brief: State 7 - Send coordinates 
# 
# description: 
#
def s7_delete_select(): 
    return 

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

    # State 0 - Operation select UI state 
    if (state == 0): 
        s0_op_select_ui() 
        state = 1 
    
    # State 1 - Operation select state 
    elif (state == 1): 
        state_return = s1_op_select(operation) 

        if (state_return == 1):     # Get 
            state = 2 
        
        elif (state_return == 2):   # Send 
            state = 4 
        
        elif (state_return == 3):   # Delete 
            state = 6 

    # State 2 - Getter application select UI state 
    elif (state == 2): 
        s2_app_select_ui() 
        state = 3 
    
    # State 3 - Getter application select state 
    elif (state == 3): 
        state_return = s3_get_select(app) 

        if (state_return == 1): 
            state = 0 
    
    # State 4 - Send coordinates UI state 
    elif (state == 4): 
        s4_send_select_ui() 
        state = 5 
    
    # State 5 - Send coordinates state 
    elif (state == 5): 
        s5_send_select() 
        state = 0 

    # State 6 - Delete coordinates UI state 
    elif (state == 6): 
        s6_delete_select_ui() 
        state = 7 
    
    # State 7 - Delete coordinates state 
    elif (state == 7): 
        s6_delete_select() 
        state = 0 

    # Default back to operation select state 
    else: 
        state = 0 
    
    #==================================================

#================================================================================
