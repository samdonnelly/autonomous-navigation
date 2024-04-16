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
#include "boat_radio_config.h" 

// Standard library 
#include <iostream> 

extern "C"
{
	// Add your C-only include files here 
}

//=======================================================================================


//=======================================================================================
// Macros 

#define NUM_TEST_RADIO_CMDS 4 

//=======================================================================================


//=======================================================================================
// Classes 

// Forward declare a mock 'Boat' class - 'Boat' is used here but it could be any vehicle 
class Boat; 


// Mock vehicle radio module - 'Boat' is used here but it could be any vehicle 
class VehicleRadio : public RadioModule<Boat, BOAT_RADIO_NUM_CMDS> 
{
public: 

    // Constructor 
    VehicleRadio() {} 

    // Destructor 
    ~VehicleRadio() {} 

    // Data 
    uint8_t cmd0_count, cmd1_count, cmd2_count, cmd3_count; 

    //==================================================
    // Commands 

    // Command callbacks 
    static void Cmd0Callback(Boat& boat_radio, uint8_t cmd0_value); 
    static void Cmd1Callback(Boat& boat_radio, uint8_t cmd1_value); 
    static void Cmd2Callback(Boat& boat_radio, uint8_t cmd2_value); 
    static void Cmd3Callback(Boat& boat_radio, uint8_t cmd3_value); 

    // Command table 
    // The value for the number of commands has to match one of the instantiations in the 
    // radio module but we can use fewer commands here. The number of commands written out 
    // here must match NUM_TEST_RADIO_CMDS. 
    std::array<RadioCmdData, BOAT_RADIO_NUM_CMDS> command_table = 
    {{
        {boat_radio_ping,   &Cmd0Callback, CLEAR_BIT}, 
        {boat_radio_idle,   &Cmd1Callback, CLEAR_BIT}, 
        {boat_radio_auto,   &Cmd2Callback, CLEAR_BIT}, 
        {boat_radio_manual, &Cmd3Callback, CLEAR_BIT} 
    }}; 
    
    //==================================================

    //==================================================
    // Class method testing functions 

    // Enable/disable commands 
    void EnableCmds(
        const std::string command, 
        uint8_t cmd_state)
    {
        CommandEnable(command, command_table, cmd_state); 
    }

    //==================================================
}; 


// Mock 'Boat' class - 'Boat' is used here but it could be any vehicle 
class Boat 
{
public: 

    // Constructor 
    Boat() {} 

    // Destructor 
    ~Boat() {} 

    // Data 
    VehicleRadio radio; 
}; 

//=======================================================================================


//=======================================================================================
// Helper functions 

// Command 0 callback 
void VehicleRadio::Cmd0Callback(Boat& boat_radio, uint8_t cmd0_value) 
{
    boat_radio.radio.cmd0_count = cmd0_value;
} 

// Command 1 callback 
void VehicleRadio::Cmd1Callback(Boat& boat_radio, uint8_t cmd1_value) 
{
    boat_radio.radio.cmd1_count = cmd1_value;
} 

// Command 2 callback 
void VehicleRadio::Cmd2Callback(Boat& boat_radio, uint8_t cmd2_value) 
{
    boat_radio.radio.cmd2_count = cmd2_value;
} 

// Command 3 callback 
void VehicleRadio::Cmd3Callback(Boat& boat_radio, uint8_t cmd3_value) 
{
    boat_radio.radio.cmd3_count = cmd3_value;
} 


//=======================================================================================


//=======================================================================================
// Test group 

TEST_GROUP(radio_module_test)
{
    // Global test group variables 
    Boat vehicle; 

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
// Command look up 

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


// Command Parse test 
// Parse a command that doesn't exist but has a valid format and check that the ID and 
// value are correct without calling a callback. 

//=======================================================================================


//=======================================================================================
// Command enable/disable 

// Enable commands then disable them 
TEST(radio_module_test, cmd_enable_disable)
{
    // Check that commands are initially disabled 
    for (uint8_t i = CLEAR; i < NUM_TEST_RADIO_CMDS; i++)
    {
        LONGS_EQUAL(CLEAR_BIT, vehicle.radio.command_table[i].cmd_enable); 
    }

    // Enable all commands (in random order) 
    vehicle.radio.EnableCmds(boat_radio_auto, SET_BIT); 
    vehicle.radio.EnableCmds(boat_radio_idle, SET_BIT); 
    vehicle.radio.EnableCmds(boat_radio_manual, SET_BIT); 
    vehicle.radio.EnableCmds(boat_radio_ping, SET_BIT); 

    // Check that all commands are enabled 
    for (uint8_t i = CLEAR; i < NUM_TEST_RADIO_CMDS; i++)
    {
        LONGS_EQUAL(SET_BIT, vehicle.radio.command_table[i].cmd_enable); 
    }

    // Disable two commands 
    vehicle.radio.EnableCmds(boat_radio_idle, CLEAR_BIT); 
    vehicle.radio.EnableCmds(boat_radio_manual, CLEAR_BIT); 

    // Check that only the two commands were disabled 
    LONGS_EQUAL(SET_BIT, vehicle.radio.command_table[0].cmd_enable); 
    LONGS_EQUAL(CLEAR_BIT, vehicle.radio.command_table[1].cmd_enable); 
    LONGS_EQUAL(SET_BIT, vehicle.radio.command_table[2].cmd_enable); 
    LONGS_EQUAL(CLEAR_BIT, vehicle.radio.command_table[3].cmd_enable); 
}

// Try enabling a command that is not available 
TEST(radio_module_test, cmd_enable_invalid)
{
    // Try enabling command that does not exist (use an existing command but starting 
    // from index 1) 
    vehicle.radio.EnableCmds(&boat_radio_auto[1], SET_BIT); 

    // Check that all commands are still disabled 
    for (uint8_t i = CLEAR; i < NUM_TEST_RADIO_CMDS; i++)
    {
        LONGS_EQUAL(CLEAR_BIT, vehicle.radio.command_table[i].cmd_enable); 
    }
}

//=======================================================================================
