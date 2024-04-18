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

// Production code 
#include "boat.h" 

// Library 
#include <iostream> 

extern "C"
{
	// Add your C-only include files here 
    #include "queue_mock.h" 
    #include "nrf24l01_driver_mock.h" 
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
    BoatUTest boat_utest; 

    // Constructor 
    void setup()
    {
        QueueMockInit(); 
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

// Command Read: Radio check event queue population 
TEST(boat_radio_test, command_read_radio_check_queue)
{
    // "Read" from the radio and queue an event to check the contents of the message. 
    // The radio is mocked to always indicate there is new data when the command read 
    // function is called. 

    boat_utest.RadioCommandRead(boat); 
    LONGS_EQUAL(boat_utest.GetMainRadioCheckEventType(), QueueMockGetNextEvent()); 
}


// Command Check: Command not enabled so no callback (using idle state command) 
TEST(boat_radio_test, command_check_not_enabled)
{
    // This test simulates a recieved radio message that matches a predefined command but 
    // the command is not enabled so no callback is used. 

    // Generate a valid idle state command radio message 
    char idle_test_str[MAX_RADIO_CMD_SIZE]; 
    uint8_t idle_test_str_len = (uint8_t)snprintf(idle_test_str, 
                                                  MAX_RADIO_CMD_SIZE, 
                                                  "%c%s", 
                                                  (char)boat_radio_idle.size(), 
                                                  boat_radio_idle.c_str()); 

    // Disable all commands, clear main thread flags and set the message to be read 
    boat_utest.RadioCommandEnable(boat, CLEAR_BIT); 
    boat_utest.ClearMainFlags(boat); 
    nrf24l01_mock_set_read_data((uint8_t *)idle_test_str, idle_test_str_len); 

    // Verify that the idle state command callback has not been called yet 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStandbyStateFlag(boat)); 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateExitFlag(boat)); 

    // Read and process the radio message 
    boat_utest.RadioCommandRead(boat); 
    boat_utest.RadioCommandCheck(boat); 

    // Check that idle state command callback was called 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStandbyStateFlag(boat)); 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateExitFlag(boat)); 
}


// Command Check: Command invalid so no callback used 
TEST(boat_radio_test, command_check_invalid_cmd)
{
    // Need a connection status flag to test this 
}


// Command Check: Heartbeat command callback 
TEST(boat_radio_test, command_check_heartbeat_callback)
{
    // This test simulates a recieved radio message that matches a predefined command and 
    // the changes made from the callback function is checked. 

    // The the heartbeat command does not currently perform any actions. It is a 
    // placeholder callback function to allow for the radio connection status to be 
    // updated. 

    // Generate a valid heartbeat command radio message 
    char heartbeat_test_str[MAX_RADIO_CMD_SIZE]; 
    uint8_t heartbeat_test_str_len = (uint8_t)snprintf(heartbeat_test_str, 
                                                       MAX_RADIO_CMD_SIZE, 
                                                       "%c%s", 
                                                       (char)boat_radio_RP.size(), 
                                                       boat_radio_RP.c_str()); 

    // Enable all commands, clear main thread flags and set the message to be read 
    boat_utest.RadioCommandEnable(boat, SET_BIT); 
    boat_utest.ClearMainFlags(boat); 
    nrf24l01_mock_set_read_data((uint8_t *)heartbeat_test_str, heartbeat_test_str_len); 

    // Verify that the heartbeat command callback has not been called yet 

    // Read and process the radio message 
    boat_utest.RadioCommandRead(boat); 
    boat_utest.RadioCommandCheck(boat); 

    // Check that heartbeat command callback was called 
}


// Command Check: Idle state command callback 
TEST(boat_radio_test, command_check_idle_callback)
{
    // This test simulates a recieved radio message that matches a predefined command and 
    // the changes made from the callback function is checked. 

    // Generate a valid idle state command radio message 
    char idle_test_str[MAX_RADIO_CMD_SIZE]; 
    uint8_t idle_test_str_len = (uint8_t)snprintf(idle_test_str, 
                                                  MAX_RADIO_CMD_SIZE, 
                                                  "%c%s", 
                                                  (char)boat_radio_idle.size(), 
                                                  boat_radio_idle.c_str()); 

    // Enable all commands, clear main thread flags and set the message to be read 
    boat_utest.RadioCommandEnable(boat, SET_BIT); 
    boat_utest.ClearMainFlags(boat); 
    nrf24l01_mock_set_read_data((uint8_t *)idle_test_str, idle_test_str_len); 

    // Verify that the idle state command callback has not been called yet 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStandbyStateFlag(boat)); 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateExitFlag(boat)); 

    // Read and process the radio message 
    boat_utest.RadioCommandRead(boat); 
    boat_utest.RadioCommandCheck(boat); 

    // Check that idle state command callback was called 
    LONGS_EQUAL(SET_BIT, boat_utest.GetMainStandbyStateFlag(boat)); 
    LONGS_EQUAL(SET_BIT, boat_utest.GetMainStateExitFlag(boat)); 
}


// Command Check: Auto state command callback 
TEST(boat_radio_test, command_check_auto_callback)
{
    // This test simulates a recieved radio message that matches a predefined command and 
    // the changes made from the callback function is checked. 

    // Generate a valid auto state command radio message 
    char auto_test_str[MAX_RADIO_CMD_SIZE]; 
    uint8_t auto_test_str_len = (uint8_t)snprintf(auto_test_str, 
                                                  MAX_RADIO_CMD_SIZE, 
                                                  "%c%s", 
                                                  (char)boat_radio_auto.size(), 
                                                  boat_radio_auto.c_str()); 

    // Enable all commands, clear main thread flags and set the message to be read 
    boat_utest.RadioCommandEnable(boat, SET_BIT); 
    boat_utest.ClearMainFlags(boat); 
    nrf24l01_mock_set_read_data((uint8_t *)auto_test_str, auto_test_str_len); 

    // Verify that the auto state command callback has not been called yet 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainAutoStateFlag(boat)); 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateExitFlag(boat)); 

    // Read and process the radio message 
    boat_utest.RadioCommandRead(boat); 
    boat_utest.RadioCommandCheck(boat); 

    // Check that auto state command callback was called 
    LONGS_EQUAL(SET_BIT, boat_utest.GetMainAutoStateFlag(boat)); 
    LONGS_EQUAL(SET_BIT, boat_utest.GetMainStateExitFlag(boat)); 
}


// Command Check: Manual state command callback 
TEST(boat_radio_test, command_check_manual_callback)
{
    // This test simulates a recieved radio message that matches a predefined command and 
    // the changes made from the callback function is checked. 

    // Generate a valid manual state command radio message 
    char manual_test_str[MAX_RADIO_CMD_SIZE]; 
    uint8_t manual_test_str_len = (uint8_t)snprintf(manual_test_str, 
                                                    MAX_RADIO_CMD_SIZE, 
                                                    "%c%s", 
                                                    (char)boat_radio_manual.size(), 
                                                    boat_radio_manual.c_str()); 

    // Enable all commands, clear main thread flags and set the message to be read 
    boat_utest.RadioCommandEnable(boat, SET_BIT); 
    boat_utest.ClearMainFlags(boat); 
    nrf24l01_mock_set_read_data((uint8_t *)manual_test_str, manual_test_str_len); 

    // Verify that the manual state command callback has not been called yet 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainManualStateFlag(boat)); 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateExitFlag(boat)); 

    // Read and process the radio message 
    boat_utest.RadioCommandRead(boat); 
    boat_utest.RadioCommandCheck(boat); 

    // Check that manual state command callback was called 
    LONGS_EQUAL(SET_BIT, boat_utest.GetMainManualStateFlag(boat)); 
    LONGS_EQUAL(SET_BIT, boat_utest.GetMainStateExitFlag(boat)); 
}


// Command Check: Index command callback 
TEST(boat_radio_test, command_check_index_callback)
{
    // This test simulates a recieved radio message that matches a predefined command and 
    // the changes made from the callback function is checked. 

    // Generate a valid index command radio message 
    char index_test_str[MAX_RADIO_CMD_SIZE]; 
    uint8_t index_test_str_len = (uint8_t)snprintf(index_test_str, 
                                                   MAX_RADIO_CMD_SIZE, 
                                                   "%c%s %s", 
                                                   (char)boat_radio_index.size(), 
                                                   boat_radio_index.c_str(), 
                                                   "11"); 

    // Enable all commands, clear main thread flags and set the message to be read 
    boat_utest.RadioCommandEnable(boat, SET_BIT); 
    boat_utest.ClearMainFlags(boat); 
    nrf24l01_mock_set_read_data((uint8_t *)index_test_str, index_test_str_len); 

    // Verify that the index command callback has not been called yet 

    // Read and process the radio message 
    boat_utest.RadioCommandRead(boat); 
    boat_utest.RadioCommandCheck(boat); 

    // Check that index command callback was called 
}


// Command Check: Throttle input command callback 
TEST(boat_radio_test, command_check_throttle_input_callback)
{
    // This test simulates a recieved radio message that matches a predefined command and 
    // the changes made from the callback function is checked. 

    // Generate a valid throttle command radio message 
    char throttle_test_str[MAX_RADIO_CMD_SIZE]; 
    uint8_t throttle_test_str_len = (uint8_t)snprintf(throttle_test_str, 
                                                      MAX_RADIO_CMD_SIZE, 
                                                      "%c%s %s", 
                                                      (char)boat_radio_RP.size(), 
                                                      boat_radio_RP.c_str(), 
                                                      "60"); 

    // Enable all commands, clear main thread flags and set the message to be read 
    boat_utest.RadioCommandEnable(boat, SET_BIT); 
    boat_utest.ClearMainFlags(boat); 
    nrf24l01_mock_set_read_data((uint8_t *)throttle_test_str, throttle_test_str_len); 

    // Verify that the throttle command callback has not been called yet 

    // Read and process the radio message 
    boat_utest.RadioCommandRead(boat); 
    boat_utest.RadioCommandCheck(boat); 

    // Check that throttle command callback was called 
}

//=======================================================================================
