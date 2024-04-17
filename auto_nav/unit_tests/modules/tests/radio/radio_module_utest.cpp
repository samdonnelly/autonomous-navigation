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
    uint8_t cmd0_value, cmd1_value, cmd2_value, cmd3_value; 

    //==================================================
    // Commands 

    // Command callbacks 
    static void Cmd0Callback(Boat& boat_radio, uint8_t command0_value); 
    static void Cmd1Callback(Boat& boat_radio, uint8_t command1_value); 
    static void Cmd2Callback(Boat& boat_radio, uint8_t command2_value); 
    static void Cmd3Callback(Boat& boat_radio, uint8_t command3_value); 

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

    // Match command input to pre-defined commands (ID + value) 
    uint8_t LookUpCmds(
        const char *command_str, 
        Boat& vehicle)
    {
        return CommandLookUp((uint8_t *)command_str, command_table, vehicle); 
    }

    // Enable/disable commands 
    void EnableCmds(
        const std::string command, 
        uint8_t cmd_state)
    {
        CommandEnable(command, command_table, cmd_state); 
    }

    // Get last recorded command ID 
    void GetCmdID(std::string& command_id)
    {
        command_id = (char *)cmd_id; 
    }

    // Get last recorded command value 
    uint8_t GetCmdValue(void)
    {
        return cmd_value; 
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
void VehicleRadio::Cmd0Callback(Boat& boat_radio, uint8_t command0_value) 
{
    boat_radio.radio.cmd0_value = command0_value; 
} 

// Command 1 callback 
void VehicleRadio::Cmd1Callback(Boat& boat_radio, uint8_t command1_value) 
{
    boat_radio.radio.cmd1_value = command1_value; 
} 

// Command 2 callback 
void VehicleRadio::Cmd2Callback(Boat& boat_radio, uint8_t command2_value) 
{
    boat_radio.radio.cmd2_value = command2_value; 
} 

// Command 3 callback 
void VehicleRadio::Cmd3Callback(Boat& boat_radio, uint8_t command3_value) 
{
    boat_radio.radio.cmd3_value = command3_value; 
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


//=======================================================================================
// Command look up 

// Command Parse test 
TEST(radio_module_test, cmd_parse)
{
    // This test looks at what gets stored in the 'cmd_id' and 'cmd_value' variables 
    // in the radio module for various command inputs. 

    // Test command inputs 
    std::string 
    test_str0 = boat_radio_ping,     // Valid ID with no value (set to zero) 
    test_str1 = boat_radio_ping, 
    test_str2 = boat_radio_idle, 
    test_str3 = boat_radio_auto, 
    test_str4 = boat_radio_manual,   
    test_str5 = "yoyo~",             // Invalid character in command ID 
    test_str6 = " 100",              // First character space will return empty command 
    test_str7 = " @00",              // Empty ID and invalid value 
    test_str8 = "eh 138  ";          // Valid with spaces after that get ignored 

    // Modify the predefined strings with values 
    test_str1.append(" ");           // Space after will have not affect ID 
    test_str2.append(" 88");         // Normal 
    test_str3.append("  88");        // Double space will skip the value 
    test_str4.append(" 8^8");        // Invalid digit character 

    // Store the info from the last parsed command input 
    std::string last_command; 
    uint8_t last_value; 

    // Test string 0 
    vehicle.radio.LookUpCmds(test_str0.c_str(), vehicle); 
    vehicle.radio.GetCmdID(last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    STRCMP_EQUAL(boat_radio_ping.c_str(), last_command.c_str()); 
    LONGS_EQUAL(0, last_value); 

    // Test string 1 
    vehicle.radio.LookUpCmds(test_str1.c_str(), vehicle); 
    vehicle.radio.GetCmdID(last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    STRCMP_EQUAL(boat_radio_ping.c_str(), last_command.c_str()); 
    LONGS_EQUAL(0, last_value); 

    // Test string 2 
    vehicle.radio.LookUpCmds(test_str2.c_str(), vehicle); 
    vehicle.radio.GetCmdID(last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    STRCMP_EQUAL(boat_radio_idle.c_str(), last_command.c_str()); 
    LONGS_EQUAL(88, last_value); 

    // Test string 3 
    vehicle.radio.LookUpCmds(test_str3.c_str(), vehicle); 
    vehicle.radio.GetCmdID(last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    STRCMP_EQUAL(boat_radio_auto.c_str(), last_command.c_str()); 
    LONGS_EQUAL(0, last_value); 

    // Test string 4 
    vehicle.radio.LookUpCmds(test_str4.c_str(), vehicle); 
    vehicle.radio.GetCmdID(last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    STRCMP_EQUAL(boat_radio_manual.c_str(), last_command.c_str()); 
    LONGS_EQUAL(0, last_value); 

    // Test string 5 
    vehicle.radio.LookUpCmds(test_str5.c_str(), vehicle); 
    vehicle.radio.GetCmdID(last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    STRCMP_EQUAL("yoyo", last_command.c_str()); 
    LONGS_EQUAL(0, last_value); 

    // Test string 6 
    vehicle.radio.LookUpCmds(test_str6.c_str(), vehicle); 
    vehicle.radio.GetCmdID(last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    STRCMP_EQUAL("", last_command.c_str()); 
    LONGS_EQUAL(100, last_value); 

    // Test string 7 
    vehicle.radio.LookUpCmds(test_str7.c_str(), vehicle); 
    vehicle.radio.GetCmdID(last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    STRCMP_EQUAL("", last_command.c_str()); 
    LONGS_EQUAL(0, last_value); 

    // Test string 8 
    vehicle.radio.LookUpCmds(test_str8.c_str(), vehicle); 
    vehicle.radio.GetCmdID(last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    STRCMP_EQUAL("eh", last_command.c_str()); 
    LONGS_EQUAL(138, last_value); 
}


// Test callback but also that non-enabled callbacks won't work 
TEST(radio_module_test, cmd_callback)
{
    // Define sample commands (all valid) 
    std::string 
    test_str0 = boat_radio_ping, 
    test_str1 = boat_radio_idle, 
    test_str2 = boat_radio_auto, 
    test_str3 = boat_radio_manual; 
    test_str0.append(" 11"); 
    test_str1.append(" 22"); 
    test_str2.append(" 33"); 
    test_str3.append(" 44"); 

    // Check that a valid command with a non-enabled callback won't work 
    LONGS_EQUAL(FALSE, vehicle.radio.LookUpCmds(test_str0.c_str(), vehicle)); 
    vehicle.radio.LookUpCmds(test_str1.c_str(), vehicle); 
    vehicle.radio.LookUpCmds(test_str2.c_str(), vehicle); 
    vehicle.radio.LookUpCmds(test_str3.c_str(), vehicle); 
    LONGS_EQUAL(0, vehicle.radio.cmd0_value); 
    LONGS_EQUAL(0, vehicle.radio.cmd1_value); 
    LONGS_EQUAL(0, vehicle.radio.cmd2_value); 
    LONGS_EQUAL(0, vehicle.radio.cmd3_value); 

    // Enable all predefined commands 
    vehicle.radio.EnableCmds(boat_radio_ping, SET_BIT); 
    vehicle.radio.EnableCmds(boat_radio_idle, SET_BIT); 
    vehicle.radio.EnableCmds(boat_radio_auto, SET_BIT); 
    vehicle.radio.EnableCmds(boat_radio_manual, SET_BIT); 

    // Check that a valid command with an enabled callback will work 
    vehicle.radio.LookUpCmds(test_str0.c_str(), vehicle); 
    vehicle.radio.LookUpCmds(test_str1.c_str(), vehicle); 
    vehicle.radio.LookUpCmds(test_str2.c_str(), vehicle); 
    vehicle.radio.LookUpCmds(test_str3.c_str(), vehicle); 
    LONGS_EQUAL(11, vehicle.radio.cmd0_value); 
    LONGS_EQUAL(22, vehicle.radio.cmd1_value); 
    LONGS_EQUAL(33, vehicle.radio.cmd2_value); 
    LONGS_EQUAL(44, vehicle.radio.cmd3_value); 
}

//=======================================================================================
