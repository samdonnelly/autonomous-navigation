/**
 * @file project_init.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Ground station initialization header 
 * 
 * @version 0.1
 * @date 2023-07-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _GS_INIT_H_ 
#define _GS_INIT_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "gs_includes_app.h"
#include "includes_drivers.h"

//=======================================================================================


//=======================================================================================
// Functions 

/**
 * @brief Ground station initialization 
 * 
 * @details 
 */
void gs_init(void); 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _GS_INIT_H_ 
