/**
 * @file boat_radio.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat radio interface 
 * 
 * @version 0.1
 * @date 2024-04-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _BOAT_RADIO_H_
#define _BOAT_RADIO_H_ 

//=======================================================================================
// Includes 

#include "radio_module.h" 
#include "vehicle_radio_config.h" 
#include "nrf24l01_driver.h" 

//=======================================================================================


//=======================================================================================
// Classes 

class BoatRadio : public RadioModule<Boat, VEHICLE_RADIO_NUM_CMDS> 
{
private:   // Private members 

    // nRF24L01 module data 
    nrf24l01_data_pipe_t data_pipe;                // Payload data pipe 
    const nrf24l01_data_pipe_t gs_pipe;            // Ground station (GS) data pipe 
    uint8_t read_buff[NRF24L01_MAX_PAYLOAD_LEN];   // Data buffer in PRX mode 
    uint8_t *write_ptr;                            // Pointer to write buffer in PTX mode 

    // General radio data 
    uint8_t gs_connect_flag;                       // Ground station connection status 
    uint8_t gs_connect_timeout;                    // Ground station connection timeout 

private:   // Private member functions 

    //==================================================
    // Commands 

    // Command callbacks 
    static void HBCmd(Boat& boat_radio, uint8_t *hb_cmd_arg); 
    static void IdleCmd(Boat& boat_radio, uint8_t *idle_cmd_arg); 
    static void AutoCmd(Boat& boat_radio, uint8_t *auto_cmd_arg); 
    static void ManualCmd(Boat& boat_radio, uint8_t *manual_cmd_arg); 
    static void IndexCmd(Boat& boat_radio, uint8_t *index_cmd_arg); 
    static void ThrottleCmd(Boat& boat_radio, uint8_t *throttle_cmd_arg); 

    // Command table 
    std::array<RadioCmdData, VEHICLE_RADIO_NUM_CMDS> command_table = 
    {{
        // Ground station commands 
        {vehicle_radio_cmd_ping,   CMD_ARG_NONE,  &HBCmd,       CLEAR_BIT}, 
        {vehicle_radio_cmd_idle,   CMD_ARG_NONE,  &IdleCmd,     CLEAR_BIT}, 
        {vehicle_radio_cmd_auto,   CMD_ARG_NONE,  &AutoCmd,     CLEAR_BIT}, 
        {vehicle_radio_cmd_manual, CMD_ARG_NONE,  &ManualCmd,   CLEAR_BIT}, 
        {vehicle_radio_cmd_index,  CMD_ARG_VALUE, &IndexCmd,    CLEAR_BIT}, 
        {vehicle_radio_cmd_RP,     CMD_ARG_VALUE, &ThrottleCmd, CLEAR_BIT}, 
        {vehicle_radio_cmd_RN,     CMD_ARG_VALUE, &ThrottleCmd, CLEAR_BIT}, 
        {vehicle_radio_cmd_RM,     CMD_ARG_VALUE, &ThrottleCmd, CLEAR_BIT}, 
        {vehicle_radio_cmd_LP,     CMD_ARG_VALUE, &ThrottleCmd, CLEAR_BIT}, 
        {vehicle_radio_cmd_LN,     CMD_ARG_VALUE, &ThrottleCmd, CLEAR_BIT}, 
        {vehicle_radio_cmd_LM,     CMD_ARG_VALUE, &ThrottleCmd, CLEAR_BIT} 
    }}; 
    
    //==================================================
    

public:   // Public member functions 

    // Constructor(s) 
    BoatRadio(nrf24l01_data_pipe_t gs_data_pipe) 
        : data_pipe(NRF24L01_RX_FIFO_EMPTY), 
          gs_pipe(gs_data_pipe), 
          gs_connect_flag(CLEAR_BIT), 
          gs_connect_timeout(CLEAR) {} 

    // Destructor 
    ~BoatRadio() {} 

    // Command handling 
    void CommandRead(Boat& boat_radio); 
    void CommandCheck(Boat& boat_radio); 
    void CommandSet(Boat& boat_radio, const char *command); 
    void CommandSend(void); 

    // Status 
    uint8_t ConnectionStatus(void); 

    // Command enable/disable 
    void MainStandbyStateCmdEnable(uint8_t cmd_state); 
    void MainAutoStateCmdEnable(uint8_t cmd_state); 
    void MainManualStateCmdEnable(uint8_t cmd_state); 
    void MainLowPwrStateCmdEnable(uint8_t cmd_state); 
    void MainFaultStateCmdEnable(uint8_t cmd_state); 
}; 

//=======================================================================================

#endif   // _BOAT_RADIO_H_ 
