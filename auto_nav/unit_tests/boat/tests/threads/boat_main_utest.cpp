/**
 * @file boat_main_utest.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat main thread unit tests 
 * 
 * @version 0.1
 * @date 2024-03-19
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

#include "boat.h" 

extern "C"
{
	// Add your C-only include files here 
    #include "queue_mock.h" 
}

//=======================================================================================


//=======================================================================================
// Macros 
//=======================================================================================


//=======================================================================================
// Test group 

TEST_GROUP(boat_main_test)
{
    // Global test group variables 
    BoatUTest boat_utest; 

    // Constructor 
    void setup()
    {
        boat_utest.ClearMainStateFlags(boat); 
        boat_utest.SetMainNoEvent(); 
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
// State machine test 

// These tests test the state machine transitions to other states. They do not test how 
// and why a state transition occurs. They all run state with "NO_EVENT" set so nothing 
// else is run. State transitions are checked from lowest priority to highest priority 
// to check that a higher priority transition will happen even with multiple state flags 
// set. 

// State machine: Init state 
TEST(boat_main_test, boat_main_state_machine_init_state)
{
    // Check init state 
    boat_utest.SetMainInitState(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainInitStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 

    // Check standby state transition 
    boat_utest.SetMainInitState(boat); 
    boat_utest.SetMainStandbyStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainStandbyStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 

    // Check fault state transition 
    boat_utest.SetMainInitState(boat); 
    boat_utest.SetMainFaultStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainFaultStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 
}


// State machine: Standby state 
TEST(boat_main_test, boat_main_state_machine_standby_state)
{
    // Check standby state 
    boat_utest.SetMainStandbyState(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainStandbyStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 

    // Check manual state 
    boat_utest.SetMainStandbyState(boat); 
    boat_utest.SetMainManualStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainManualStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 

    // Check auto state 
    boat_utest.SetMainStandbyState(boat); 
    boat_utest.SetMainAutoStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainAutoStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 

    // Check low power state 
    boat_utest.SetMainStandbyState(boat); 
    boat_utest.SetMainLowPwrStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainLowPwrStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 

    // Check fault state 
    boat_utest.SetMainStandbyState(boat); 
    boat_utest.SetMainFaultStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainFaultStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 
}


// State machine: Auto state 
TEST(boat_main_test, boat_main_state_machine_auto_state)
{
    // Check auto state 
    boat_utest.SetMainAutoState(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainAutoStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 

    // Check manual state 
    boat_utest.SetMainAutoState(boat); 
    boat_utest.SetMainManualStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainManualStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 

    // Check standby state 
    boat_utest.SetMainAutoState(boat); 
    boat_utest.SetMainStandbyStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainStandbyStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 

    // Check low power state 
    boat_utest.SetMainAutoState(boat); 
    boat_utest.SetMainLowPwrStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainLowPwrStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 

    // Check fault state 
    boat_utest.SetMainAutoState(boat); 
    boat_utest.SetMainFaultStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainFaultStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 
}


// State machine: Manual state 
TEST(boat_main_test, boat_main_state_machine_manual_state)
{
    // Check manual state 
    boat_utest.SetMainManualState(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainManualStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 

    // Check auto state 
    boat_utest.SetMainManualState(boat); 
    boat_utest.SetMainAutoStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainAutoStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 

    // Check standby state 
    boat_utest.SetMainManualState(boat); 
    boat_utest.SetMainStandbyStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainStandbyStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 

    // Check low power state 
    boat_utest.SetMainManualState(boat); 
    boat_utest.SetMainLowPwrStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainLowPwrStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 

    // Check fault state 
    boat_utest.SetMainManualState(boat); 
    boat_utest.SetMainFaultStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainFaultStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 
}


// State machine: Low Power state 
TEST(boat_main_test, boat_main_state_machine_low_pwr_state)
{
    // Check low power state 
    boat_utest.SetMainLowPwrState(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainLowPwrStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 

    // Check reset state 
    boat_utest.SetMainLowPwrState(boat); 
    boat_utest.SetMainResetStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainResetStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 
    
    // Check standby state 
    boat_utest.SetMainLowPwrState(boat); 
    boat_utest.SetMainStandbyStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainStandbyStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 
}


// State machine: Fault state 
TEST(boat_main_test, boat_main_state_machine_fault_state)
{
    // Check fault state 
    boat_utest.SetMainFaultState(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainFaultStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 

    // Check reset state 
    boat_utest.SetMainFaultState(boat); 
    boat_utest.SetMainResetStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainResetStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 
}


// State machine: Reset state 
TEST(boat_main_test, boat_main_state_machine_reset_state)
{
    // Check reset state 
    boat_utest.SetMainResetState(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainResetStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 

    // Check init state 
    boat_utest.SetMainResetState(boat); 
    boat_utest.SetMainInitStateFlag(boat); 
    boat_utest.MainThreadDispatch(boat); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetMainInitStateType(), 
                         boat_utest.GetMainCurrentState(boat)); 
}

//=======================================================================================


//=======================================================================================
// State entry/exit test 

// These tests test the entry and exit of each state in the main thread. They do not test 
// how and why a state enters or exits. They all run with a "NO_EVENT" event so nothing 
// else takes place. 

// State: Init 
TEST(boat_main_test, boat_main_state_enter_exit_init_state)
{
    // State entry check 
    boat_utest.SetMainStateEntryFlag(boat); 
    boat_utest.SetMainInitStateFlag(boat); 
    boat_utest.MainInitStateWrapper(boat); 
    UNSIGNED_LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateEntryFlag(boat)); 
    UNSIGNED_LONGS_EQUAL(SET_BIT, boat_utest.GetMainInitStateFlag(boat)); 

    // State exit check 
    boat_utest.SetMainStateExitFlag(boat); 
    boat_utest.MainInitStateWrapper(boat); 
    UNSIGNED_LONGS_EQUAL(SET_BIT, boat_utest.GetMainStateEntryFlag(boat)); 
    UNSIGNED_LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainInitStateFlag(boat)); 
}


// State: Standby 
TEST(boat_main_test, boat_main_state_enter_exit_standby_state)
{
    // State entry check 
    boat_utest.SetMainStateEntryFlag(boat); 
    boat_utest.SetMainStandbyStateFlag(boat); 
    boat_utest.MainStandbyStateWrapper(boat); 
    UNSIGNED_LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateEntryFlag(boat)); 
    UNSIGNED_LONGS_EQUAL(SET_BIT, boat_utest.GetMainStandbyStateFlag(boat)); 

    // State exit check 
    boat_utest.SetMainStateExitFlag(boat); 
    boat_utest.MainStandbyStateWrapper(boat); 
    UNSIGNED_LONGS_EQUAL(SET_BIT, boat_utest.GetMainStateEntryFlag(boat)); 
    UNSIGNED_LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStandbyStateFlag(boat)); 

    // Check queue was filled correctly 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetCommsLEDStrobeOffEventType(), QueueMockGetNextEvent()); 
}


// State: Auto 
TEST(boat_main_test, boat_main_state_enter_exit_auto_state)
{
    // State entry check 
    boat_utest.SetMainStateEntryFlag(boat); 
    boat_utest.SetMainAutoStateFlag(boat); 
    boat_utest.MainAutoStateWrapper(boat); 
    UNSIGNED_LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateEntryFlag(boat)); 
    UNSIGNED_LONGS_EQUAL(SET_BIT, boat_utest.GetMainAutoStateFlag(boat)); 

    // State exit check 
    boat_utest.SetMainStateExitFlag(boat); 
    boat_utest.MainAutoStateWrapper(boat); 
    UNSIGNED_LONGS_EQUAL(SET_BIT, boat_utest.GetMainStateEntryFlag(boat)); 
    UNSIGNED_LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainAutoStateFlag(boat)); 

    // Check queue was filled correctly 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetCommsLEDWriteEventType(), QueueMockGetNextEvent()); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetCommsLEDStrobeOffEventType(), QueueMockGetNextEvent()); 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetCommsLEDWriteEventType(), QueueMockGetNextEvent()); 
}


// State: Manual 
TEST(boat_main_test, boat_main_state_enter_exit_manual_state)
{
    // State entry check 
    boat_utest.SetMainStateEntryFlag(boat); 
    boat_utest.SetMainManualStateFlag(boat); 
    boat_utest.MainManualStateWrapper(boat); 
    UNSIGNED_LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateEntryFlag(boat)); 
    UNSIGNED_LONGS_EQUAL(SET_BIT, boat_utest.GetMainManualStateFlag(boat)); 

    // State exit check 
    boat_utest.SetMainStateExitFlag(boat); 
    boat_utest.MainManualStateWrapper(boat); 
    UNSIGNED_LONGS_EQUAL(SET_BIT, boat_utest.GetMainStateEntryFlag(boat)); 
    UNSIGNED_LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainManualStateFlag(boat)); 

    // Check queue was filled correctly 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetCommsLEDStrobeOffEventType(), QueueMockGetNextEvent()); 
}


// State: Low Power 
TEST(boat_main_test, boat_main_state_enter_exit_low_pwr_state)
{
    // State entry check 
    boat_utest.SetMainStateEntryFlag(boat); 
    boat_utest.SetMainLowPwrStateFlag(boat); 
    boat_utest.MainLowPwrStateWrapper(boat); 
    UNSIGNED_LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateEntryFlag(boat)); 
    UNSIGNED_LONGS_EQUAL(SET_BIT, boat_utest.GetMainLowPwrStateFlag(boat)); 

    // State exit check 
    boat_utest.SetMainStateExitFlag(boat); 
    boat_utest.MainLowPwrStateWrapper(boat); 
    UNSIGNED_LONGS_EQUAL(SET_BIT, boat_utest.GetMainStateEntryFlag(boat)); 
    UNSIGNED_LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainLowPwrStateFlag(boat)); 

    // Check queue was filled correctly 
    UNSIGNED_LONGS_EQUAL(boat_utest.GetCommsLEDStrobeOffEventType(), QueueMockGetNextEvent()); 
}


// State: Fault 
TEST(boat_main_test, boat_main_state_enter_exit_fault_state)
{
    // State entry check 
    boat_utest.SetMainStateEntryFlag(boat); 
    boat_utest.SetMainFaultStateFlag(boat); 
    boat_utest.MainFaultStateWrapper(boat); 
    UNSIGNED_LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateEntryFlag(boat)); 
    UNSIGNED_LONGS_EQUAL(SET_BIT, boat_utest.GetMainFaultStateFlag(boat)); 

    // State exit check 
    boat_utest.SetMainStateExitFlag(boat); 
    boat_utest.MainFaultStateWrapper(boat); 
    UNSIGNED_LONGS_EQUAL(SET_BIT, boat_utest.GetMainStateEntryFlag(boat)); 
    UNSIGNED_LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainFaultStateFlag(boat)); 
}


// State: Reset 
TEST(boat_main_test, boat_main_state_enter_exit_reset_state)
{
    // State entry check 
    boat_utest.SetMainStateEntryFlag(boat); 
    boat_utest.SetMainResetStateFlag(boat); 
    boat_utest.MainResetStateWrapper(boat); 
    UNSIGNED_LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainStateEntryFlag(boat)); 
    UNSIGNED_LONGS_EQUAL(SET_BIT, boat_utest.GetMainResetStateFlag(boat)); 

    // State exit check 
    boat_utest.SetMainStateExitFlag(boat); 
    boat_utest.MainResetStateWrapper(boat); 
    UNSIGNED_LONGS_EQUAL(SET_BIT, boat_utest.GetMainStateEntryFlag(boat)); 
    UNSIGNED_LONGS_EQUAL(CLEAR_BIT, boat_utest.GetMainResetStateFlag(boat)); 
}

//=======================================================================================


//=======================================================================================
// State event test 

// These tests test the events available in each state of the main thread. They do not 
// test how and why state event is requested. 

// State: Init 
TEST(boat_main_test, boat_main_state_events_init_state)
{
    // 
}


// State: Standby 
TEST(boat_main_test, boat_main_state_events_standby_state)
{
    // 
}


// State: Auto 
TEST(boat_main_test, boat_main_state_events_auto_state)
{
    // 
}


// State: Manual 
TEST(boat_main_test, boat_main_state_events_manual_state)
{
    // 
}


// State: Low Power 
TEST(boat_main_test, boat_main_state_events_low_pwr_state)
{
    // 
}


// State: Fault 
TEST(boat_main_test, boat_main_state_events_fault_state)
{
    // 
}


// State: Reset 
TEST(boat_main_test, boat_main_state_events_reset_state)
{
    // 
}

//=======================================================================================
