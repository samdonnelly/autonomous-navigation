/**
 * @file radio_module_utest.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Radio module unit tests 
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

// Production code 
#include "radio_module.h" 
#include "boat.h" 

extern "C"
{
	// Add your C-only include files here 
}

//=======================================================================================


//=======================================================================================
// Macros 

#define NUM_CMD_MSGS 4 

//=======================================================================================


//=======================================================================================
// Classes 

// Mock vehicle radio module - 'Boat' is used here but it could be any vehicle 
class VehicleRadio : public RadioModule<Boat, NUM_CMD_MSGS> 
{
private: 

    // Command table 
    std::array<RadioCmdData, NUM_CMD_MSGS> command_table = 
    {{
        // {boat_radio_ping,   &HBCmd,       CLEAR_BIT}, 
        // {boat_radio_idle,   &IdleCmd,     CLEAR_BIT}, 
        // {boat_radio_auto,   &AutoCmd,     CLEAR_BIT}, 
        // {boat_radio_manual, &ManualCmd,   CLEAR_BIT}, 
        // {boat_radio_index,  &IndexCmd,    CLEAR_BIT}, 
        // {boat_radio_RP,     &ThrottleCmd, CLEAR_BIT}, 
        // {boat_radio_RN,     &ThrottleCmd, CLEAR_BIT}, 
        // {boat_radio_LP,     &ThrottleCmd, CLEAR_BIT}, 
        // {boat_radio_LN,     &ThrottleCmd, CLEAR_BIT} 
    }}; 
}

//=======================================================================================


//=======================================================================================
// Test group 

TEST_GROUP(radio_module_test)
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
// State machine test 

// Test 0 
TEST(radio_module_test, test0)
{
    std::string test_str = "string"; 
    std::string str0 = "strin"; 
    std::string str1 = "string"; 
    std::string str2 = "strings"; 

    LONGS_EQUAL(false, test_str.compare(str0) == 0); 
    LONGS_EQUAL(true, test_str.compare(str1) == 0); 
    LONGS_EQUAL(false, test_str.compare(str2) == 0); 
}

//=======================================================================================
