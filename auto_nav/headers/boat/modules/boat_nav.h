/**
 * @file boat_nav.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat navigation interface 
 * 
 * @version 0.1
 * @date 2024-06-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _BOAT_NAV_H_
#define _BOAT_NAV_H_ 

//=======================================================================================
// Includes 

#include "nav_module.h" 

//=======================================================================================


//=======================================================================================
// Classes 

class BoatNav : public NavModule 
{
private:   // Private members 

    // 

private:   // Private member functions 

    // 

public:   // Public member functions 

    // Constructor 
    BoatNav() 
        : NavModule(100) {}

    // Destructor 
    ~BoatNav() {}

    // Navigation 
    void HeadingUpdate(void); 
    void HeadingCalc(void); 
    void LocationUpdate(void); 
    void LocationCalc(void); 
}; 

//=======================================================================================

#endif   // _BOAT_NAV_H_ 
