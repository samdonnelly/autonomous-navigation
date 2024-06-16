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

// Get size of char string 
uint8_t get_str_size(const char *string)
{
    uint8_t size = CLEAR; 

    while (*string++ != NULL_CHAR)
    {
        size++; 
    }

    return size; 
}

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


// Command Read and Check: Radio connection lost and heartbeat command callback 
TEST(boat_radio_test, command_read_check_timeout_heartbeat)
{
    // This test simulates a loss of radio connection and checks for the heartbeat 
    // callback function. If a valid command is not received from the ground station 
    // after a certain period of time then the connection flag is cleared to indicate 
    // there is no radio connection. No radio connection is simulated by repeatedly 
    // reading a command that does not match any pre-defined commands which therefore 
    // allows the timeout to run out. The heartbeat callback function does not perform 
    // any actions as the only pupose of the heartbeat command is to match with one 
    // of the pre-defined commands so the connection flag can be set. After getting 
    // the connection flag to clear, we can verify that the heartbeat callback was 
    // executed by observing a change in the connection flag. 

    // Invalid command to set (won't match with a pre-defined command) 
    char invalid_cmd[] = "pinger"; 

    // Buffer to get the heartbeat callback response 
    uint8_t send_buffer[NRF24L01_MAX_PAYLOAD_LEN]; 

    // Enable all commands 
    boat_utest.RadioCommandEnable(boat, SET_BIT); 

    // Set a valid message to read, then read it to get a true radio connection status. 
    nrf24l01_mock_set_read_data((uint8_t *)boat_radio_cmd_ping, 
                                get_str_size(boat_radio_cmd_ping)); 
    boat_utest.RadioCommandRead(boat); 
    boat_utest.RadioCommandCheck(boat); 
    LONGS_EQUAL(SET_BIT, boat_utest.RadioConnectionStatus(boat)); 

    // Update the read message to an invalid command then check for a valid command 
    // until the connection times out. Make sure to check enough times to exceed the 
    // timeout. 
    nrf24l01_mock_set_read_data((uint8_t *)invalid_cmd, (uint8_t)sizeof(invalid_cmd)); 

    for (uint8_t i = CLEAR; i < 100; i++)
    {
        boat_utest.RadioCommandRead(boat); 
        boat_utest.RadioCommandCheck(boat); 
    }

    LONGS_EQUAL(CLEAR_BIT, boat_utest.RadioConnectionStatus(boat)); 

    // Now that the connection is lost, reestablish the connection using the heartbeat 
    // command to verify the callback. Also check for a heartbeat (ping) response. 
    nrf24l01_mock_set_read_data((uint8_t *)boat_radio_cmd_ping, 
                                get_str_size(boat_radio_cmd_ping)); 
    boat_utest.RadioCommandRead(boat); 
    boat_utest.RadioCommandCheck(boat); 
    LONGS_EQUAL(SET_BIT, boat_utest.RadioConnectionStatus(boat)); 
    
    boat_utest.RadioCommandSend(boat); 
    nrf24l01_mock_get_send_data(send_buffer); 
    STRCMP_EQUAL(boat_radio_ping_confirm, (char *)send_buffer); 
}


// Command Check: Command not enabled so no callback (using "idle" command) 
TEST(boat_radio_test, command_check_not_enabled)
{
    // This test simulates a recieved radio message that matches a predefined command but 
    // the command is not enabled so no callback is used. The "idle" command is used for 
    // this and the next test shows the expected result of the "idle" command callback 
    // once it is enabled. 

    // Disable all commands, clear main thread flags and set the message to be read 
    boat_utest.RadioCommandEnable(boat, CLEAR_BIT); 
    boat_utest.ClearMainFlags(boat); 
    nrf24l01_mock_set_read_data((uint8_t *)boat_radio_cmd_idle, 
                                get_str_size(boat_radio_cmd_idle)); 

    // Verify that the idle state command callback has not been called yet 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStandbyStateFlag(boat)); 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateExitFlag(boat)); 
    LONGS_EQUAL(255, QueueMockGetNextEvent()); 

    // Read and process the radio message 
    boat_utest.RadioCommandRead(boat); 
    boat_utest.RadioCommandCheck(boat); 
    QueueMockGetNextEvent();   // Call this to bypass the radio check event 

    // Check that idle state command callback was still not called due to it not 
    // being enabled. 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStandbyStateFlag(boat)); 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateExitFlag(boat)); 
    LONGS_EQUAL(255, QueueMockGetNextEvent()); 
}


// Command Check: "idle" command callback 
TEST(boat_radio_test, command_check_idle_callback)
{
    // This test simulates a recieved radio message that matches a predefined command and 
    // the changes made from the callback function is checked. 

    uint8_t send_buffer[NRF24L01_MAX_PAYLOAD_LEN]; 

    // Enable all commands, clear main thread flags and set the message to be read 
    boat_utest.RadioCommandEnable(boat, SET_BIT); 
    boat_utest.ClearMainFlags(boat); 
    nrf24l01_mock_set_read_data((uint8_t *)boat_radio_cmd_idle, 
                                get_str_size(boat_radio_cmd_idle)); 

    // Verify that the idle state command callback has not been called yet 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStandbyStateFlag(boat)); 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateExitFlag(boat)); 
    LONGS_EQUAL(255, QueueMockGetNextEvent()); 

    // Read and process the radio message 
    boat_utest.RadioCommandRead(boat); 
    boat_utest.RadioCommandCheck(boat); 
    QueueMockGetNextEvent();   // Call this to bypass the radio check event 

    // Check that idle state command callback was called. See the callback function for 
    // items checked here. 
    LONGS_EQUAL(SET_BIT, boat_utest.GetMainStandbyStateFlag(boat)); 
    LONGS_EQUAL(SET_BIT, boat_utest.GetMainStateExitFlag(boat)); 
    LONGS_EQUAL(boat_utest.GetMainNoEventType(), QueueMockGetNextEvent()); 
    LONGS_EQUAL(boat_utest.GetCommsRadioSendEventType(), QueueMockGetNextEvent()); 
    boat_utest.RadioCommandSend(boat); 
    nrf24l01_mock_get_send_data(send_buffer); 
    STRCMP_EQUAL(boat_radio_msg_confirm, (char *)send_buffer); 
}


// Command Check: "auto" command callback 
TEST(boat_radio_test, command_check_auto_callback)
{
    // This test simulates a recieved radio message that matches a predefined command and 
    // the changes made from the callback function is checked. 

    uint8_t send_buffer[NRF24L01_MAX_PAYLOAD_LEN]; 

    // Enable all commands, clear main thread flags and set the message to be read 
    boat_utest.RadioCommandEnable(boat, SET_BIT); 
    boat_utest.ClearMainFlags(boat); 
    nrf24l01_mock_set_read_data((uint8_t *)boat_radio_cmd_auto, 
                                get_str_size(boat_radio_cmd_auto)); 

    // Verify that the auto command callback has not been called yet. A GPS connection 
    // is simulated to help show that the callback has not been called. 
    boat_utest.NavNavstatSet(boat, SET_BIT); 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainAutoStateFlag(boat)); 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateExitFlag(boat)); 
    LONGS_EQUAL(255, QueueMockGetNextEvent());   // 255 means no event queued 

    // Read and process the radio message. The GPS connection is simulated to have been 
    // lost to show that we can't enter the auto state without having a connection. 
    boat_utest.NavNavstatSet(boat, CLEAR_BIT); 
    boat_utest.RadioCommandRead(boat); 
    boat_utest.RadioCommandCheck(boat); 
    QueueMockGetNextEvent();   // Call this to bypass the radio check event 

    // Check that the auto command callback did not trigger the auto state due to 
    // there being no position lock (navstat not set). 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainAutoStateFlag(boat)); 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateExitFlag(boat)); 
    LONGS_EQUAL(255, QueueMockGetNextEvent()); 

    // Repeat the process but this time simulate a GPS connection by setting 'navstat' 
    // so that the auto state command callback can trigger the auto state. 
    boat_utest.NavNavstatSet(boat, SET_BIT); 
    boat_utest.RadioCommandRead(boat); 
    boat_utest.RadioCommandCheck(boat); 
    QueueMockGetNextEvent(); 
    
    LONGS_EQUAL(SET_BIT, boat_utest.GetMainAutoStateFlag(boat)); 
    LONGS_EQUAL(SET_BIT, boat_utest.GetMainStateExitFlag(boat)); 
    LONGS_EQUAL(boat_utest.GetMainNoEventType(), QueueMockGetNextEvent()); 
    LONGS_EQUAL(boat_utest.GetCommsRadioSendEventType(), QueueMockGetNextEvent()); 
    boat_utest.RadioCommandSend(boat); 
    nrf24l01_mock_get_send_data(send_buffer); 
    STRCMP_EQUAL(boat_radio_msg_confirm, (char *)send_buffer); 
}


// Command Check: "manual" command callback 
TEST(boat_radio_test, command_check_manual_callback)
{
    // This test simulates a recieved radio message that matches a predefined command and 
    // the changes made from the callback function is checked. 

    uint8_t send_buffer[NRF24L01_MAX_PAYLOAD_LEN]; 

    // Enable all commands, clear main thread flags and set the message to be read 
    boat_utest.RadioCommandEnable(boat, SET_BIT); 
    boat_utest.ClearMainFlags(boat); 
    nrf24l01_mock_set_read_data((uint8_t *)boat_radio_cmd_manual, 
                                get_str_size(boat_radio_cmd_manual)); 

    // Verify that the manual command callback has not been called yet 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainManualStateFlag(boat)); 
    LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateExitFlag(boat)); 
    LONGS_EQUAL(255, QueueMockGetNextEvent()); 

    // Read and process the radio message 
    boat_utest.RadioCommandRead(boat); 
    boat_utest.RadioCommandCheck(boat); 
    QueueMockGetNextEvent();   // Call this to bypass the radio check event 

    // Check that manual command callback was executed. See the callback function for 
    // items checked here. 
    LONGS_EQUAL(SET_BIT, boat_utest.GetMainManualStateFlag(boat)); 
    LONGS_EQUAL(SET_BIT, boat_utest.GetMainStateExitFlag(boat)); 
    LONGS_EQUAL(boat_utest.GetMainNoEventType(), QueueMockGetNextEvent()); 
    LONGS_EQUAL(boat_utest.GetCommsRadioSendEventType(), QueueMockGetNextEvent()); 
    boat_utest.RadioCommandSend(boat); 
    nrf24l01_mock_get_send_data(send_buffer); 
    STRCMP_EQUAL(boat_radio_msg_confirm, (char *)send_buffer); 
}


// Command Check: "index" command callback 
TEST(boat_radio_test, command_check_index_callback)
{
    // This test simulates a recieved radio message that matches a predefined command and 
    // the changes made from the callback function is checked. 

    // Create the sample index commands 
    char index_cmd_0[10], index_cmd_1[10]; 
    strcpy(index_cmd_0, boat_radio_cmd_index); 
    strcat(index_cmd_0, " 10");   // Within the waypoint mission index range 
    strcpy(index_cmd_1, boat_radio_cmd_index); 
    strcat(index_cmd_1, " 5");   // Outside the waypoint mission index range 

    // Buffer to get the radio send message 
    uint8_t send_buffer[NRF24L01_MAX_PAYLOAD_LEN]; 

    // Set the number of waypoints in the mission 
    boat_utest.NavLoadMission(boat); 

    // Enable all commands, clear main thread flags and set the initial message to be 
    // read. The initial message is an index command with an index outside the range 
    // of the total number of waypoints in the mission. 
    boat_utest.RadioCommandEnable(boat, SET_BIT); 
    boat_utest.ClearMainFlags(boat); 
    nrf24l01_mock_set_read_data((uint8_t *)index_cmd_0, get_str_size(index_cmd_0)); 

    // Verify that the index command callback has not been called yet 
    LONGS_EQUAL(255, QueueMockGetNextEvent()); 

    // Read and process the radio message 
    boat_utest.RadioCommandRead(boat); 
    boat_utest.RadioCommandCheck(boat); 
    QueueMockGetNextEvent();   // Call this to bypass the radio check event 

    // The command read was valid but the index was out of range so the waypoint 
    // mission index was not updated and therefore no confirmation message was 
    // sent back (meaning no radio send event was queued). 
    LONGS_EQUAL(255, QueueMockGetNextEvent()); 

    // Update the command to an index command that contains an index within range of 
    // the mission waypoints. After this, read the command again and verify the 
    // callback was executed. 
    nrf24l01_mock_set_read_data((uint8_t *)index_cmd_1, get_str_size(index_cmd_1)); 
    boat_utest.RadioCommandRead(boat); 
    boat_utest.RadioCommandCheck(boat); 
    QueueMockGetNextEvent();   // Call this to bypass the radio check event 
    LONGS_EQUAL(boat_utest.GetCommsRadioSendEventType(), QueueMockGetNextEvent()); 
    boat_utest.RadioCommandSend(boat); 
    nrf24l01_mock_get_send_data(send_buffer); 
    STRCMP_EQUAL(boat_radio_msg_confirm, (char *)send_buffer); 
}


// Command Check: Throttle command callback 
TEST(boat_radio_test, command_check_throttle_input_callback)
{
    // This test simulates a recieved radio message that matches a predefined command and 
    // the changes made from the callback function is checked. 

    // Enable all commands, clear main thread flags and set the message to be read 
    boat_utest.RadioCommandEnable(boat, SET_BIT); 
    boat_utest.ClearMainFlags(boat); 
    nrf24l01_mock_set_read_data((uint8_t *)boat_radio_cmd_RP, 
                                get_str_size(boat_radio_cmd_RP)); 

    // Verify that the throttle command callback has not been called yet 

    // Read and process the radio message 
    boat_utest.RadioCommandRead(boat); 
    boat_utest.RadioCommandCheck(boat); 
    QueueMockGetNextEvent();   // Call this to bypass the radio check event 

    // Check that throttle command callback was executed. See the callback function for 
    // items checked here. 
}

//=======================================================================================
