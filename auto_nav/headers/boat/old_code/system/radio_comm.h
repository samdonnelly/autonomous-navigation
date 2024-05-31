/**
 * @file radio_comm.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief AB radio control interface 
 * 
 * @version 0.1
 * @date 2024-03-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _AB_RADIO_COMM_H_ 
#define _AB_RADIO_COMM_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "includes_drivers.h" 

//=======================================================================================


//=======================================================================================
// Macros 

#define AB_PL_LEN 32                 // Payload length 
#define AB_MAX_CMD_SIZE 32           // Max external command size 

//=======================================================================================


//=======================================================================================
// Classes 

class boat_radio_comms 
{
private:   // Private members 

    // Timing information 
    TIM_TypeDef *timer_nonblocking;          // Timer used for non-blocking delays 
    uint8_t hb_timeout;                      // Heartbeat timeout count 

    // Radio info 
    nrf24l01_data_pipe_t pipe;               // Data pipe number for the radio module 

    // Payload data 
    uint8_t read_buff[AB_PL_LEN];            // Data read by PRX from PTX device 

    // Flags 
    uint8_t connect_flag;                    // Connect flag 

public:   // Public members 

    // Timing 
    tim_compare_t hb_timer;                  // Heartbeat timing info 
    
    // Payload data 
    uint8_t cmd_id[AB_MAX_CMD_SIZE];         // Stores the ID of the external command 
    uint8_t cmd_value;                       // Stores the value of the external command 

public:   // Public member functions 

    // Constructor 
    boat_radio_comms(TIM_TypeDef *timer); 

    // Destructor 
    ~boat_radio_comms(); 

    /**
     * @brief Checks for radio communication 
     * 
     * @details Checks the radio heartbeat to see if the boat can still talk to the ground 
     *          station, and checks if there are any new commands to be read. If there is 
     *          new input then the input is checked against the predefined commands and if 
     *          a macth is found then the command function is called. 
     */
    void radio_comm_check(uint8_t state); 

    /**
     * @brief Connection status 
     * 
     * @return uint8_t : 'connect_flag' - used to indicate radio connection 
     */
    uint8_t connect_status(void); 

private:   // Private member functions 

    /**
     * @brief Parse the user command into an ID and value 
     * 
     * @details Takes a radio message received from the ground station and parses it into 
     *          an ID and payload. If the ID and payload are of a valid format then the 
     *          function will return true. Note that a payload is not needed for all 
     *          commands. See the 'cmd_table' for a list of available commands/IDs and the 
     *          states in which they're used. 
     * 
     * @param command_buffer : radio message string 
     * @return uint8_t : status of the message parsing 
     */
    uint8_t command_parse(uint8_t *cmd_buff); 
}; 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _AB_RADIO_COMM_H_ 
