/**
 * @file boat_utest.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat unit testing 
 * 
 * @version 0.1
 * @date 2024-03-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _BOAT_UTEST_H_
#define _BOAT_UTEST_H_ 

//=======================================================================================
// Includes 

#include "includes_drivers.h" 

//=======================================================================================


//=======================================================================================
// Classes 

// Forward declare Boat class 
class Boat; 

// This class is for Boat unit testing only. Do not define these methods anywhere in the 
// application code. 
class BoatUTest 
{
public: 

    //==================================================
    // Main thread 

    // Data 
    Event main_event; 

    // Wrappers 
    void MainThreadDispatch(Boat& boat_utest); 
    void MainInitStateWrapper(Boat& boat_utest); 
    void MainStandbyStateWrapper(Boat& boat_utest); 
    void MainAutoStateWrapper(Boat& boat_utest); 
    void MainManualStateWrapper(Boat& boat_utest); 
    void MainLowPwrStateWrapper(Boat& boat_utest); 
    void MainFaultStateWrapper(Boat& boat_utest); 
    void MainResetStateWrapper(Boat& boat_utest); 

    // Current state getter 
    uint8_t GetMainCurrentState(Boat& boat_utest); 

    // State type getters 
    uint8_t GetMainInitStateType(void); 
    uint8_t GetMainStandbyStateType(void); 
    uint8_t GetMainAutoStateType(void); 
    uint8_t GetMainManualStateType(void); 
    uint8_t GetMainLowPwrStateType(void); 
    uint8_t GetMainFaultStateType(void); 
    uint8_t GetMainResetStateType(void); 

    // Event type getters 
    uint8_t GetMainNoEventType(void); 
    uint8_t GetMainInitEventType(void); 
    uint8_t GetMainRadioCheckEventType(void); 
    uint8_t GetMainNavCalcsEventType(void); 
    uint8_t GetMainRemoteControlEventType(void); 

    // Main thread flag getters 
    uint8_t GetMainStateEntryFlag(Boat& boat_utest); 
    uint8_t GetMainStateExitFlag(Boat& boat_utest); 
    uint8_t GetMainInitStateFlag(Boat& boat_utest); 
    uint8_t GetMainStandbyStateFlag(Boat& boat_utest); 
    uint8_t GetMainAutoStateFlag(Boat& boat_utest); 
    uint8_t GetMainManualStateFlag(Boat& boat_utest); 
    uint8_t GetMainLowPwrStateFlag(Boat& boat_utest); 
    uint8_t GetMainFaultStateFlag(Boat& boat_utest); 
    uint8_t GetMainResetStateFlag(Boat& boat_utest); 

    // Current state setters 
    void SetMainInitState(Boat& boat_utest); 
    void SetMainStandbyState(Boat& boat_utest); 
    void SetMainAutoState(Boat& boat_utest); 
    void SetMainManualState(Boat& boat_utest); 
    void SetMainLowPwrState(Boat& boat_utest); 
    void SetMainFaultState(Boat& boat_utest); 
    void SetMainResetState(Boat& boat_utest); 

    // Main thread flag setters 
    void SetMainStateEntryFlag(Boat& boat_utest); 
    void SetMainStateExitFlag(Boat& boat_utest); 
    void SetMainInitStateFlag(Boat& boat_utest); 
    void SetMainStandbyStateFlag(Boat& boat_utest); 
    void SetMainAutoStateFlag(Boat& boat_utest); 
    void SetMainManualStateFlag(Boat& boat_utest); 
    void SetMainLowPwrStateFlag(Boat& boat_utest); 
    void SetMainFaultStateFlag(Boat& boat_utest); 
    void SetMainResetStateFlag(Boat& boat_utest); 
    void ClearMainFlags(Boat& boat_utest); 

    // Event setters 
    void SetMainNoEvent(void); 
        
    //==================================================

    //==================================================
    // Comms thread 

    // Event type getters 
    uint8_t GetCommsNoEventType(void); 
    uint8_t GetCommsLEDStrobeEventType(void); 
    uint8_t GetCommsLEDStrobeOffEventType(void); 
    uint8_t GetCommsLEDWriteEventType(void); 
    uint8_t GetCommsRadioReadEventType(void); 

    //==================================================

    //==================================================
    // Boat radio module 

    // Command read and check 
    void RadioCommandRead(Boat& boat_utest); 
    void RadioCommandCheck(Boat& boat_utest); 
    void RadioCommandEnable(
        Boat& boat_utest, 
        uint8_t cmd_state); 

    //==================================================

    // Constructor 
    BoatUTest() {} 

    // Destructor 
    ~BoatUTest() {} 
}; 

//=======================================================================================

#endif   // _BOAT_UTEST_H_ 
