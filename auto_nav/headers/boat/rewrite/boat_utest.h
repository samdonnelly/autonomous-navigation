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

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "boat.h" 

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

    // Getters 
    uint8_t GetCurrentState(Boat& boat_utest); 
    uint8_t GetInitStateFlag(Boat& boat_utest); 
    uint8_t GetStandbyStateFlag(Boat& boat_utest); 
    uint8_t GetAutoStateFlag(Boat& boat_utest); 
    uint8_t GetManualStateFlag(Boat& boat_utest); 
    uint8_t GetLowPwrStateFlag(Boat& boat_utest); 
    uint8_t GetFaultStateFlag(Boat& boat_utest); 
    uint8_t GetResetStateFlag(Boat& boat_utest); 

    // Setters 
    void SetInitStateFlag(Boat& boat_utest); 
    void SetStandbyStateFlag(Boat& boat_utest); 
    void SetAutoStateFlag(Boat& boat_utest); 
    void SetManualStateFlag(Boat& boat_utest); 
    void SetLowPwrStateFlag(Boat& boat_utest); 
    void SetFaultStateFlag(Boat& boat_utest); 
    void SetResetStateFlag(Boat& boat_utest); 
        
    //==================================================

private: 

    // 
}; 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _BOAT_UTEST_H_ 
