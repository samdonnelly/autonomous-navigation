/**
 * @file boat_utest.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat unit testing definitions 
 * 
 * @details The functions from BoatUTest are defined here. These should not be redefined 
 *          in the application code anywhere nor should they be used by any application 
 *          code. They're for unit testing only. 
 * 
 * @version 0.1
 * @date 2024-03-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "boat.h" 

//=======================================================================================


//=======================================================================================
// Boat redefinition for unit testing 

Boat boat; 

Boat::Boat() 
    : leds(DEVICE_ONE, 0, 0), 
      radio(NRF24L01_DP_1) {} 

//=======================================================================================


//=======================================================================================
// Main thread 

//==================================================
// Wrappers 

// Dispatch function (from event loop) 
void BoatUTest::MainThreadDispatch(Boat& boat_utest)
{
    boat_utest.BoatMainDispatch(main_event); 
}

// Init state 
void BoatUTest::MainInitStateWrapper(Boat& boat_utest)
{
    boat_utest.MainInitState(boat_utest, main_event); 
}

// Standby state 
void BoatUTest::MainStandbyStateWrapper(Boat& boat_utest)
{
    boat_utest.MainStandbyState(boat_utest, main_event); 
}

// Auto state 
void BoatUTest::MainAutoStateWrapper(Boat& boat_utest)
{
    boat_utest.MainAutoState(boat_utest, main_event); 
}

// Manual state 
void BoatUTest::MainManualStateWrapper(Boat& boat_utest)
{
    boat_utest.MainManualState(boat_utest, main_event); 
}

// Low Power state 
void BoatUTest::MainLowPwrStateWrapper(Boat& boat_utest)
{
    boat_utest.MainLowPwrState(boat_utest, main_event); 
}

// Fault state 
void BoatUTest::MainFaultStateWrapper(Boat& boat_utest)
{
    boat_utest.MainFaultState(boat_utest, main_event); 
}

// Reset state 
void BoatUTest::MainResetStateWrapper(Boat& boat_utest)
{
    boat_utest.MainResetState(boat_utest, main_event); 
}

//==================================================

//==================================================
// Current state getter 

// State getter 
uint8_t BoatUTest::GetMainCurrentState(Boat& boat_utest)
{
    return (uint8_t)boat_utest.main_state; 
}

//==================================================

//==================================================
// State type getters 

// Get init state value 
uint8_t BoatUTest::GetMainInitStateType(void)
{
    return (uint8_t)Boat::MainStates::INIT_STATE; 
}

// Get standby state value 
uint8_t BoatUTest::GetMainStandbyStateType(void)
{
    return (uint8_t)Boat::MainStates::STANDBY_STATE; 
}

// Get auto state value 
uint8_t BoatUTest::GetMainAutoStateType(void)
{
    return (uint8_t)Boat::MainStates::AUTO_STATE; 
}

// Get manual state value 
uint8_t BoatUTest::GetMainManualStateType(void)
{
    return (uint8_t)Boat::MainStates::MANUAL_STATE; 
}

// Get low power state value 
uint8_t BoatUTest::GetMainLowPwrStateType(void)
{
    return (uint8_t)Boat::MainStates::LOW_PWR_STATE; 
}

// Get fault state value 
uint8_t BoatUTest::GetMainFaultStateType(void)
{
    return (uint8_t)Boat::MainStates::FAULT_STATE; 
}

// Get reset state value 
uint8_t BoatUTest::GetMainResetStateType(void)
{
    return (uint8_t)Boat::MainStates::RESET_STATE; 
}

//==================================================

//==================================================
// Event type getters 

// Get NO_EVENT value 
uint8_t BoatUTest::GetMainNoEventType(void)
{
    return (uint8_t)Boat::MainEvents::NO_EVENT; 
}

// Get INIT value 
uint8_t BoatUTest::GetMainInitEventType(void)
{
    return (uint8_t)Boat::MainEvents::INIT; 
}

// Get RADIO_CHECK value 
uint8_t BoatUTest::GetMainRadioCheckEventType(void)
{
    return (uint8_t)Boat::MainEvents::RADIO_CHECK; 
}

// Get NAV_HEADING_CALC value 
uint8_t BoatUTest::GetMainNavHeadingCalcEventType(void)
{
    return (uint8_t)Boat::MainEvents::NAV_HEADING_CALC; 
}

// Get NAV_LOCATION_CALC value 
uint8_t BoatUTest::GetMainNavLocationCalcEventType(void)
{
    return (uint8_t)Boat::MainEvents::NAV_LOCATION_CALC; 
}

// Get REMOTE_CONTROL value 
uint8_t BoatUTest::GetMainRemoteControlEventType(void)
{
    return (uint8_t)Boat::MainEvents::REMOTE_CONTROL; 
}

//==================================================

//==================================================
// Main thread flag getters 

// State entry flag getter 
uint8_t BoatUTest::GetMainStateEntryFlag(Boat& boat_utest)
{
    return boat_utest.main_flags.state_entry; 
}

// State exit flag getter 
uint8_t BoatUTest::GetMainStateExitFlag(Boat& boat_utest)
{
    return boat_utest.main_flags.state_exit; 
}

// Init state flag getter 
uint8_t BoatUTest::GetMainInitStateFlag(Boat& boat_utest)
{
    return boat_utest.main_flags.init_state; 
}

// Standby state flag getter 
uint8_t BoatUTest::GetMainStandbyStateFlag(Boat& boat_utest)
{
    return boat_utest.main_flags.standby_state; 
}

// Auto state flag getter 
uint8_t BoatUTest::GetMainAutoStateFlag(Boat& boat_utest)
{
    return boat_utest.main_flags.auto_state; 
}

// Manual state flag getter 
uint8_t BoatUTest::GetMainManualStateFlag(Boat& boat_utest)
{
    return boat_utest.main_flags.manual_state; 
}

// Low power state flag getter 
uint8_t BoatUTest::GetMainLowPwrStateFlag(Boat& boat_utest)
{
    return boat_utest.main_flags.low_pwr_state; 
}

// Fault state flag getter 
uint8_t BoatUTest::GetMainFaultStateFlag(Boat& boat_utest)
{
    return boat_utest.main_flags.fault_state; 
}

// Reset state flag getter 
uint8_t BoatUTest::GetMainResetStateFlag(Boat& boat_utest)
{
    return boat_utest.main_flags.reset_state; 
}

//==================================================

//==================================================
// Current state setters 

// Set to the init state 
void BoatUTest::SetMainInitState(Boat& boat_utest)
{
    boat_utest.main_state = Boat::MainStates::INIT_STATE; 
}

// Set to the standby state 
void BoatUTest::SetMainStandbyState(Boat& boat_utest)
{
    boat_utest.main_state = Boat::MainStates::STANDBY_STATE; 
}

// Set to the auto state 
void BoatUTest::SetMainAutoState(Boat& boat_utest)
{
    boat_utest.main_state = Boat::MainStates::AUTO_STATE; 
}

// Set to the manual state 
void BoatUTest::SetMainManualState(Boat& boat_utest)
{
    boat_utest.main_state = Boat::MainStates::MANUAL_STATE; 
}

// Set to the low power state 
void BoatUTest::SetMainLowPwrState(Boat& boat_utest)
{
    boat_utest.main_state = Boat::MainStates::LOW_PWR_STATE; 
}

// Set to the fault state 
void BoatUTest::SetMainFaultState(Boat& boat_utest)
{
    boat_utest.main_state = Boat::MainStates::FAULT_STATE; 
}

// Set to the rest state 
void BoatUTest::SetMainResetState(Boat& boat_utest)
{
    boat_utest.main_state = Boat::MainStates::RESET_STATE; 
}

//==================================================

//==================================================
// Main thread flag setters 

// State entry flag setter 
void BoatUTest::SetMainStateEntryFlag(Boat& boat_utest)
{
    boat_utest.main_flags.state_entry = SET_BIT; 
}

// State exit flag setter 
void BoatUTest::SetMainStateExitFlag(Boat& boat_utest)
{
    boat_utest.main_flags.state_exit = SET_BIT; 
}

// Init state flag setter 
void BoatUTest::SetMainInitStateFlag(Boat& boat_utest)
{
    boat_utest.main_flags.init_state = SET_BIT; 
}

// Standby state flag setter 
void BoatUTest::SetMainStandbyStateFlag(Boat& boat_utest)
{
    boat_utest.main_flags.standby_state = SET_BIT; 
}

// Auto state flag setter 
void BoatUTest::SetMainAutoStateFlag(Boat& boat_utest)
{
    boat_utest.main_flags.auto_state = SET_BIT; 
}

// Manual state flag setter 
void BoatUTest::SetMainManualStateFlag(Boat& boat_utest)
{
    boat_utest.main_flags.manual_state = SET_BIT; 
}

// Low power state flag setter 
void BoatUTest::SetMainLowPwrStateFlag(Boat& boat_utest)
{
    boat_utest.main_flags.low_pwr_state = SET_BIT; 
}

// Fault state flag setter 
void BoatUTest::SetMainFaultStateFlag(Boat& boat_utest)
{
    boat_utest.main_flags.fault_state = SET_BIT; 
}

// Reset state flag setter 
void BoatUTest::SetMainResetStateFlag(Boat& boat_utest)
{
    boat_utest.main_flags.reset_state = SET_BIT; 
}

// Clear all flags 
void BoatUTest::ClearMainFlags(Boat& boat_utest)
{
    memset((void*)&boat_utest.main_flags, CLEAR, sizeof(boat_utest.main_flags)); 
}

//==================================================

//==================================================
// Event setters 

// Set event to NO_EVENT 
void BoatUTest::SetMainNoEvent(void)
{
    main_event = (Event)Boat::MainEvents::NO_EVENT; 
}

//==================================================

//=======================================================================================


//=======================================================================================
// Comms thread 

//==================================================
// Event type getters 

// Get NO_EVENT value 
uint8_t BoatUTest::GetCommsNoEventType(void)
{
    return (uint8_t)Boat::CommsEvents::NO_EVENT; 
}

// Get LED_STROBE value 
uint8_t BoatUTest::GetCommsLEDStrobeEventType(void)
{
    return (uint8_t)Boat::CommsEvents::LED_STROBE; 
}

// Get LED_STROBE_OFF value 
uint8_t BoatUTest::GetCommsLEDStrobeOffEventType(void)
{
    return (uint8_t)Boat::CommsEvents::LED_STROBE_OFF; 
}

// Get LED_WRITE value 
uint8_t BoatUTest::GetCommsLEDWriteEventType(void)
{
    return (uint8_t)Boat::CommsEvents::LED_WRITE; 
}

// Get RADIO_READ value 
uint8_t BoatUTest::GetCommsRadioReadEventType(void)
{
    return (uint8_t)Boat::CommsEvents::RADIO_READ; 
}

// Get RADIO_SEND value 
uint8_t BoatUTest::GetCommsRadioSendEventType(void)
{
    return (uint8_t)Boat::CommsEvents::RADIO_SEND; 
}

// Get NAV_HEADING_UPDATE value 
uint8_t BoatUTest::GetCommsNavHeadingUpdateEventType(void)
{
    return (uint8_t)Boat::CommsEvents::NAV_HEADING_UPDATE; 
}

// Get NAV_LOCATION_UPDATE value 
uint8_t BoatUTest::GetCommsNavLocationUpdateEventType(void)
{
    return (uint8_t)Boat::CommsEvents::NAV_LOCATION_UPDATE; 
}

//==================================================

//=======================================================================================


//=======================================================================================
// Boat radio module 

// Command Read 
void BoatUTest::RadioCommandRead(Boat& boat_utest)
{
    boat_utest.radio.CommandRead(boat_utest); 
}

// Command Check 
void BoatUTest::RadioCommandCheck(Boat& boat_utest)
{
    boat_utest.radio.CommandCheck(boat_utest); 
}

// Connection status 
uint8_t BoatUTest::RadioConnectionStatus(Boat& boat_utest)
{
    return boat_utest.radio.ConnectionStatus(); 
}

// Command Send 
void BoatUTest::RadioCommandSend(Boat& boat_utest)
{
    boat_utest.radio.CommandSend(); 
}

// Command Enable/Disable 
void BoatUTest::RadioCommandEnable(
    Boat& boat_utest, 
    uint8_t cmd_state)
{
    boat_utest.radio.MainStandbyStateCmdEnable(cmd_state); 
    boat_utest.radio.MainManualStateCmdEnable(cmd_state); 
}

//=======================================================================================


//=======================================================================================
// Boat navigation moudle 

// Update navigation status 
void BoatUTest::NavNavstatSet(
    Boat& boat_utest, 
    uint8_t status)
{
    boat_utest.navigation.navstat = status; 
}

//=======================================================================================
