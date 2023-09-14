#================================================================================
# File: 
# 
# Author: 
# 
# Description: 
# 
# Date: 
#================================================================================

#================================================================================
# Notes 
#================================================================================


#================================================================================
# Includes 
#================================================================================


#================================================================================
# Functions 

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
        return False, -1 
    
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

#================================================================================
