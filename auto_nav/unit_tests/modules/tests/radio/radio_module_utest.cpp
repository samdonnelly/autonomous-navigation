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
#include "commands_config.h" 

// Standard library 
#include <iostream> 

extern "C"
{
	// Add your C-only include files here 
}

//=======================================================================================


//=======================================================================================
// Macros 

#define ARG_STR_SIZE 30 

//=======================================================================================


//=======================================================================================
// Classes 

// Forward declare a mock 'Boat' class - 'Boat' is used here but it could be any vehicle 
class Boat; 


// Mock vehicle radio module - 'Boat' is used here but it could be any vehicle 
class VehicleRadio : public RadioModule<Boat, VEHICLE_RADIO_NUM_CMDS> 
{
public: 

    // Constructor 
    VehicleRadio() {} 

    // Destructor 
    ~VehicleRadio() {} 

    // Data 
    uint8_t cmd0_arg, cmd1_arg, cmd2_arg; 
    uint8_t cmd3_arg[ARG_STR_SIZE]; 

    //==================================================
    // Commands 

    // Command callbacks 
    static void Cmd0Callback(Boat& boat_radio, uint8_t *command0_arg); 
    static void Cmd1Callback(Boat& boat_radio, uint8_t *command1_arg); 
    static void Cmd2Callback(Boat& boat_radio, uint8_t *command2_arg); 
    static void Cmd3Callback(Boat& boat_radio, uint8_t *command3_arg); 

    // Command table 
    // This command table must match one of the instantiations in the radio module in size. 
    // The details within can be different. 
    std::array<RadioCmdData, VEHICLE_RADIO_NUM_CMDS> mock_command_table = 
    {{
        {vehicle_radio_cmd_ping,   CMD_ARG_NONE,  &Cmd0Callback, CLEAR_BIT}, 
        {vehicle_radio_cmd_idle,   CMD_ARG_VALUE, &Cmd1Callback, CLEAR_BIT}, 
        {vehicle_radio_cmd_auto,   CMD_ARG_VALUE, &Cmd2Callback, CLEAR_BIT}, 
        {vehicle_radio_cmd_manual, CMD_ARG_STR,   &Cmd3Callback, CLEAR_BIT}, 
        {vehicle_radio_cmd_index,  CMD_ARG_VALUE, &Cmd3Callback, CLEAR_BIT}, 
        {vehicle_radio_cmd_RP,     CMD_ARG_VALUE, &Cmd3Callback, CLEAR_BIT}, 
        {vehicle_radio_cmd_RN,     CMD_ARG_VALUE, &Cmd3Callback, CLEAR_BIT}, 
        {vehicle_radio_cmd_LP,     CMD_ARG_VALUE, &Cmd3Callback, CLEAR_BIT}, 
        {vehicle_radio_cmd_LN,     CMD_ARG_VALUE, &Cmd3Callback, CLEAR_BIT} 
    }}; 
    
    //==================================================

    //==================================================
    // Class method testing functions 

    // Match command input to pre-defined commands (ID + value) 
    uint8_t LookUpCmds(
        const char *command_str, 
        Boat& vehicle)
    {
        return CommandLookUp((uint8_t *)command_str, mock_command_table, vehicle); 
    }

    // Enable/disable commands 
    void EnableCmds(
        const char *command, 
        uint8_t cmd_state)
    {
        CommandEnable(command, mock_command_table, cmd_state); 
    }

    // Get last recorded command ID 
    void GetCmdID(char **command_id)
    {
        *command_id = (char *)cmd_id; 
    }

    // Get last recorded command value 
    uint8_t GetCmdValue(void)
    {
        return cmd_value; 
    }

    // Get last recorded command string 
    void GetCmdString(std::string& command_str)
    {
        command_str = (char *)cmd_str; 
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

// Command 0 callback - no argument used 
void VehicleRadio::Cmd0Callback(
    Boat& boat_radio, 
    uint8_t *command0_arg) 
{
    if (command0_arg != nullptr)
    {
        boat_radio.radio.cmd0_arg = *command0_arg; 
    }
} 

// Command 1 callback - argument is a value 
void VehicleRadio::Cmd1Callback(
    Boat& boat_radio, 
    uint8_t *command1_arg) 
{
    if (command1_arg != nullptr)
    {
        boat_radio.radio.cmd1_arg = *command1_arg; 
    }
} 

// Command 2 callback - argument is a value 
void VehicleRadio::Cmd2Callback(
    Boat& boat_radio, 
    uint8_t *command2_arg) 
{
    if (command2_arg != nullptr)
    {
        boat_radio.radio.cmd2_arg = *command2_arg; 
    }
} 

// Command 3 callback - argument is a string 
void VehicleRadio::Cmd3Callback(
    Boat& boat_radio, 
    uint8_t *command3_arg) 
{
    for (uint8_t i = CLEAR; *command3_arg != NULL_CHAR; i++)
    {
        boat_radio.radio.cmd3_arg[i] = *command3_arg++; 
    }
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
    for (uint8_t i = CLEAR; i < VEHICLE_RADIO_NUM_CMDS; i++)
    {
        LONGS_EQUAL(CLEAR_BIT, vehicle.radio.mock_command_table[i].cmd_enable); 
    }

    // Enable all commands (in random order) 
    vehicle.radio.EnableCmds(vehicle_radio_cmd_auto, SET_BIT); 
    vehicle.radio.EnableCmds(vehicle_radio_cmd_idle, SET_BIT); 
    vehicle.radio.EnableCmds(vehicle_radio_cmd_manual, SET_BIT); 
    vehicle.radio.EnableCmds(vehicle_radio_cmd_ping, SET_BIT); 

    // Check that the commands are enabled (4 of them) 
    for (uint8_t i = CLEAR; i < VEHICLE_RADIO_NUM_CMDS; i++)
    {
        if (i < 4)
        {
            LONGS_EQUAL(SET_BIT, vehicle.radio.mock_command_table[i].cmd_enable); 
        }
        else 
        {
            LONGS_EQUAL(CLEAR_BIT, vehicle.radio.mock_command_table[i].cmd_enable); 
        }
    }

    // Disable two commands 
    vehicle.radio.EnableCmds(vehicle_radio_cmd_idle, CLEAR_BIT); 
    vehicle.radio.EnableCmds(vehicle_radio_cmd_manual, CLEAR_BIT); 

    // Check that only the two commands were disabled 
    LONGS_EQUAL(SET_BIT, vehicle.radio.mock_command_table[0].cmd_enable); 
    LONGS_EQUAL(CLEAR_BIT, vehicle.radio.mock_command_table[1].cmd_enable); 
    LONGS_EQUAL(SET_BIT, vehicle.radio.mock_command_table[2].cmd_enable); 
    LONGS_EQUAL(CLEAR_BIT, vehicle.radio.mock_command_table[3].cmd_enable); 
}


// Try enabling a command that is not available 
TEST(radio_module_test, cmd_enable_invalid)
{
    // Try enabling command that does not exist (use an existing command but starting 
    // from index 1) 
    vehicle.radio.EnableCmds(&vehicle_radio_cmd_auto[1], SET_BIT); 

    // Check that all commands are still disabled 
    for (uint8_t i = CLEAR; i < VEHICLE_RADIO_NUM_CMDS; i++)
    {
        LONGS_EQUAL(CLEAR_BIT, vehicle.radio.mock_command_table[i].cmd_enable); 
    }
}

//=======================================================================================


//=======================================================================================
// Command look up 

// Command Parse test 
TEST(radio_module_test, cmd_parse)
{
    // This test looks at what gets stored in 'cmd_id', 'cmd_value' and 'cmd_str' from 
    // the radio module for various command inputs. 

    // Test command inputs 
    std::string 
    test_str0 = vehicle_radio_cmd_ping,     // Valid ID with no value (set to zero) 
    test_str1 = vehicle_radio_cmd_ping, 
    test_str2 = vehicle_radio_cmd_idle, 
    test_str3 = vehicle_radio_cmd_auto, 
    test_str4 = vehicle_radio_cmd_auto, 
    test_str5 = vehicle_radio_cmd_manual, 
    test_str6 = "yoyo~",                 // Invalid character in command ID 
    test_str7 = " 100",                  // First character space will return empty command 
    test_str8 = "eh 138  ";              // Valid with spaces after that get ignored 

    // Modify the predefined strings with values 
    test_str1.append(" ");           // Space after will have no affect on ID 
    test_str2.append(" 88");         // Normal 
    test_str3.append("  88");        // Double space will skip the value 
    test_str4.append(" 8^8");        // Invalid digit character 
    test_str5.append(" 8^8");        // Valid string argument 

    // Store the info from the last parsed command input 
    char *last_command = nullptr; 
    uint8_t last_value; 
    std::string last_string; 

    // Enable all predefined commands 
    vehicle.radio.EnableCmds(vehicle_radio_cmd_ping, SET_BIT); 
    vehicle.radio.EnableCmds(vehicle_radio_cmd_idle, SET_BIT); 
    vehicle.radio.EnableCmds(vehicle_radio_cmd_auto, SET_BIT); 
    vehicle.radio.EnableCmds(vehicle_radio_cmd_manual, SET_BIT); 

    // In each test case, the command is first processed then each component of the 
    // command (ID and value or string) is retreived and checked. The way the argument 
    // gets handled is determined by the command table argument type above. 

    // Test string 0 
    LONGS_EQUAL(TRUE, vehicle.radio.LookUpCmds((char *)test_str0.c_str(), vehicle)); 
    vehicle.radio.GetCmdID(&last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    vehicle.radio.GetCmdString(last_string); 
    STRCMP_EQUAL(vehicle_radio_cmd_ping, last_command); 
    LONGS_EQUAL(0, last_value); 
    STRCMP_EQUAL("", last_string.c_str()); 

    // Test string 1 
    LONGS_EQUAL(TRUE, vehicle.radio.LookUpCmds(test_str1.c_str(), vehicle)); 
    vehicle.radio.GetCmdID(&last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    vehicle.radio.GetCmdString(last_string); 
    STRCMP_EQUAL(vehicle_radio_cmd_ping, last_command); 
    LONGS_EQUAL(0, last_value); 
    STRCMP_EQUAL("", last_string.c_str()); 

    // Test string 2 
    LONGS_EQUAL(TRUE, vehicle.radio.LookUpCmds(test_str2.c_str(), vehicle)); 
    vehicle.radio.GetCmdID(&last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    vehicle.radio.GetCmdString(last_string); 
    STRCMP_EQUAL(vehicle_radio_cmd_idle, last_command); 
    LONGS_EQUAL(88, last_value); 
    STRCMP_EQUAL("", last_string.c_str()); 

    // Test string 3 
    LONGS_EQUAL(TRUE, vehicle.radio.LookUpCmds(test_str3.c_str(), vehicle)); 
    vehicle.radio.GetCmdID(&last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    vehicle.radio.GetCmdString(last_string); 
    STRCMP_EQUAL(vehicle_radio_cmd_auto, last_command); 
    LONGS_EQUAL(0, last_value); 
    STRCMP_EQUAL("", last_string.c_str()); 

    // Test string 4 
    LONGS_EQUAL(FALSE, vehicle.radio.LookUpCmds(test_str4.c_str(), vehicle)); 
    vehicle.radio.GetCmdID(&last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    vehicle.radio.GetCmdString(last_string); 
    STRCMP_EQUAL(vehicle_radio_cmd_auto, last_command); 
    LONGS_EQUAL(0, last_value); 
    STRCMP_EQUAL("", last_string.c_str()); 

    // Test string 5 - argument treated as a string 
    LONGS_EQUAL(TRUE, vehicle.radio.LookUpCmds(test_str5.c_str(), vehicle)); 
    vehicle.radio.GetCmdID(&last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    vehicle.radio.GetCmdString(last_string); 
    STRCMP_EQUAL(vehicle_radio_cmd_manual, last_command); 
    LONGS_EQUAL(0, last_value); 
    STRCMP_EQUAL("8^8", last_string.c_str()); 

    // No command matched for the below cases so the value and string will be the same 
    // as the last test case. 

    // Test string 6 
    LONGS_EQUAL(FALSE, vehicle.radio.LookUpCmds(test_str6.c_str(), vehicle)); 
    vehicle.radio.GetCmdID(&last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    vehicle.radio.GetCmdString(last_string); 
    STRCMP_EQUAL("yoyo", last_command); 

    // Test string 7 
    LONGS_EQUAL(FALSE, vehicle.radio.LookUpCmds(test_str7.c_str(), vehicle)); 
    vehicle.radio.GetCmdID(&last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    vehicle.radio.GetCmdString(last_string); 
    STRCMP_EQUAL("", last_command); 

    // Test string 8 
    LONGS_EQUAL(FALSE, vehicle.radio.LookUpCmds(test_str8.c_str(), vehicle)); 
    vehicle.radio.GetCmdID(&last_command); 
    last_value = vehicle.radio.GetCmdValue(); 
    vehicle.radio.GetCmdString(last_string); 
    STRCMP_EQUAL("eh", last_command); 
}


// Test callback but also that non-enabled callbacks won't work 
TEST(radio_module_test, cmd_callback)
{
    // Define sample commands (all valid) 
    std::string 
    test_str0 = vehicle_radio_cmd_ping, 
    test_str1 = vehicle_radio_cmd_idle, 
    test_str2 = vehicle_radio_cmd_auto, 
    test_str3 = vehicle_radio_cmd_manual; 
    test_str0.append(" 11"); 
    test_str1.append(" 22"); 
    test_str2.append(" 33"); 
    test_str3.append(" hello there!"); 

    // Initialize all test arguments 
    vehicle.radio.cmd0_arg = CLEAR; 
    vehicle.radio.cmd1_arg = CLEAR; 
    vehicle.radio.cmd2_arg = CLEAR; 
    memset((void *)vehicle.radio.cmd3_arg, CLEAR, sizeof(vehicle.radio.cmd3_arg)); 

    // Check that a valid command with a non-enabled callback won't work 
    LONGS_EQUAL(FALSE, vehicle.radio.LookUpCmds(test_str0.c_str(), vehicle)); 
    vehicle.radio.LookUpCmds(test_str1.c_str(), vehicle); 
    vehicle.radio.LookUpCmds(test_str2.c_str(), vehicle); 
    vehicle.radio.LookUpCmds(test_str3.c_str(), vehicle); 
    LONGS_EQUAL(0, vehicle.radio.cmd0_arg); 
    LONGS_EQUAL(0, vehicle.radio.cmd1_arg); 
    LONGS_EQUAL(0, vehicle.radio.cmd2_arg); 
    STRCMP_EQUAL("", (char *)vehicle.radio.cmd3_arg); 

    // Enable all predefined commands 
    vehicle.radio.EnableCmds(vehicle_radio_cmd_ping, SET_BIT); 
    vehicle.radio.EnableCmds(vehicle_radio_cmd_idle, SET_BIT); 
    vehicle.radio.EnableCmds(vehicle_radio_cmd_auto, SET_BIT); 
    vehicle.radio.EnableCmds(vehicle_radio_cmd_manual, SET_BIT); 

    // Check that a valid command with an enabled callback will work 
    vehicle.radio.LookUpCmds(test_str0.c_str(), vehicle); 
    vehicle.radio.LookUpCmds(test_str1.c_str(), vehicle); 
    vehicle.radio.LookUpCmds(test_str2.c_str(), vehicle); 
    vehicle.radio.LookUpCmds(test_str3.c_str(), vehicle); 
    LONGS_EQUAL(0, vehicle.radio.cmd0_arg); 
    LONGS_EQUAL(22, vehicle.radio.cmd1_arg); 
    LONGS_EQUAL(33, vehicle.radio.cmd2_arg); 
    STRCMP_EQUAL("hello there!", (char *)vehicle.radio.cmd3_arg); 
}

//=======================================================================================
