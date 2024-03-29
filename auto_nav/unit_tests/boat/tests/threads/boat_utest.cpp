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

#include "boat_utest.h" 

//=======================================================================================


//=======================================================================================
// Main thread 

//==================================================
// Getters 

// State getter 
uint8_t BoatUTest::GetCurrentState(Boat& boat_utest)
{
    return (uint8_t)boat_utest.main_state; 
}

// Init state flag getter 
uint8_t BoatUTest::GetInitStateFlag(Boat& boat_utest)
{
    return boat_utest.main_flags.init_state; 
}

// Standby state flag getter 
uint8_t BoatUTest::GetStandbyStateFlag(Boat& boat_utest)
{
    return boat_utest.main_flags.standby_state; 
}

// Auto state flag getter 
uint8_t BoatUTest::GetAutoStateFlag(Boat& boat_utest)
{
    return boat_utest.main_flags.auto_state; 
}

// Manual state flag getter 
uint8_t BoatUTest::GetManualStateFlag(Boat& boat_utest)
{
    return boat_utest.main_flags.manual_state; 
}

// Low power state flag getter 
uint8_t BoatUTest::GetLowPwrStateFlag(Boat& boat_utest)
{
    return boat_utest.main_flags.low_pwr_state; 
}

// Fault state flag getter 
uint8_t BoatUTest::GetFaultStateFlag(Boat& boat_utest)
{
    return boat_utest.main_flags.fault_state; 
}

// Reset state flag getter 
uint8_t BoatUTest::GetResetStateFlag(Boat& boat_utest)
{
    return boat_utest.main_flags.reset_state; 
}

//==================================================

//==================================================
// Setters 

// Init state flag setter 
void BoatUTest::SetInitStateFlag(Boat& boat_utest)
{
    boat_utest.main_flags.init_state = SET_BIT; 
}

// Standby state flag setter 
void BoatUTest::SetStandbyStateFlag(Boat& boat_utest)
{
    boat_utest.main_flags.init_state = SET_BIT; 
}

// Auto state flag setter 
void BoatUTest::SetAutoStateFlag(Boat& boat_utest)
{
    boat_utest.main_flags.init_state = SET_BIT; 
}

// Manual state flag setter 
void BoatUTest::SetManualStateFlag(Boat& boat_utest)
{
    boat_utest.main_flags.init_state = SET_BIT; 
}

// Low power state flag setter 
void BoatUTest::SetLowPwrStateFlag(Boat& boat_utest)
{
    boat_utest.main_flags.init_state = SET_BIT; 
}

// Fault state flag setter 
void BoatUTest::SetFaultStateFlag(Boat& boat_utest)
{
    boat_utest.main_flags.init_state = SET_BIT; 
}

// Reset state flag setter 
void BoatUTest::SetResetStateFlag(Boat& boat_utest)
{
    boat_utest.main_flags.init_state = SET_BIT; 
}

//==================================================

//=======================================================================================