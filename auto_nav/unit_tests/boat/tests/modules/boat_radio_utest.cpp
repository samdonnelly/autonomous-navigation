/**
 * @file boat_radio_utest.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat radio module unit tests 
 * 
 * @version 0.1
 * @date 2024-04-11
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Notes 
//=======================================================================================


//=======================================================================================
// Includes 

#include "CppUTest/TestHarness.h" 

extern "C"
{
	// Add your C-only include files here 
}

//=======================================================================================


//=======================================================================================
// Macros 
//=======================================================================================


//=======================================================================================
// Test group 

TEST_GROUP(boat_radio_test)
{
    // Global test group variables 

    // Constructor 
    void setup()
    {
        // 
    }

    // Destructor 
    void teardown()
    {
        // 
    }
}; 

//=======================================================================================


//=======================================================================================
// Helper functions 
//=======================================================================================


//=======================================================================================
// Tests 

// Simulate read from radio and: 
// - check event queue is populated 
// - check no callback is called when all are disabled 
// - check no callback is called when an invalid input is received 
// - check each callback with a valid command and all enabled 
// Could use one of the state command enable functions to enable certain messages but 
// these are trivial and don't need rigorous testing. 

// Test 0 
TEST(boat_radio_test, test0)
{
    // 
}

//=======================================================================================
