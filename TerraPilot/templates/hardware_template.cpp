/**
 * @file hardware_template.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle hardware template 
 * 
 * @details This is a template of the hardware module implementation. It should not be 
 *          included in the build of a user project. Instead it should be copied to the 
 *          specific project and filled out with hardware specific code as needed. 
 * 
 * @version 0.1
 * @date 2025-04-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Include 

// Autopilot 
#include "hardware.h" 

//=======================================================================================


//=======================================================================================
// Initialization 

/**
 * @brief Vehicle hardware setup code 
 * 
 * @details This function is called before the autopilot scheduler starts and it provides 
 *          an opportunity for devices and peripherals to be initialized for the chosen 
 *          hardware. 
 */
void VehicleHardware::HardwareSetup(void)
{
    // 
}

//=======================================================================================


//=======================================================================================
// Actuators 

/**
 * @brief Set the motor outpur PWM 
 * 
 * @details This function is called from the autopilots main thread to set the propulsion 
 *          motor(s) PWM output during travelling modes. Two motor outputs are provided 
 *          for vehicles that use differential thrust but only throttle_1 will be used 
 *          when a vehicle is configured for one propulsion ouptut. 
 * 
 * @param throttle_1 : motor 1 PWM (1000-2000) 
 * @param throttle_2 : motor 2 PWM (optional) (1000-2000) 
 */
void VehicleHardware::PropulsionSet(
    uint16_t throttle_1, 
    uint16_t throttle_2)
{
    // 
}


/**
 * @brief Set the steering servo/motor output PWM 
 * 
 * @details This function is called from the autopilots main thread to set the steering 
 *          motor(s) PWM output during travelling modes. 
 * 
 * @param roll : roll control PWM (1000-2000) 
 * @param pitch : pitch control PWM (1000-2000) 
 * @param yaw : yaw control PWM (1000-2000) 
 */
void VehicleHardware::SteeringSet(
    uint16_t roll, 
    uint16_t pitch, 
    uint16_t yaw)
{
    // 
}

//=======================================================================================


//=======================================================================================
// GPS 

/**
 * @brief Read from a GPS device 
 * 
 * @details This function is called periodically from the autopilots communication thread 
 *          to read new data from aGPSU device. Data is read here but not retreived by 
 *          the autopilot. Instead, the data_ready.gps_ready flag should be set if new 
 *          data is available and the autopilot will call GPSGet to collect the data 
 *          during the main thread. 
 * 
 * @see GPSGet 
 */
void VehicleHardware::GPSRead(void)
{
    // 
}


/**
 * @brief Get the data read from a GPS device 
 * 
 * @details This function is called from the autopilots main thread to collect new data 
 *          read from a GPS device if it's available. Data is copied here but not read. 
 *          This function is only called if the data_ready.gps_ready flag is set in the 
 *          GPSRead function. 
 *          
 *          The latitude, longitude and altitude must be supplied in both unsigned int 
 *          and float forms as both are used by the autopilot. The GPS position lock 
 *          status must also be returned. 
 *          
 *          Integer coordinates are expressed in degrees*10^7 to maintain precision. 
 *          Float coordinates are expressed in degrees with decimal values. 
 * 
 * @see GPSRead 
 * 
 * @param location : latitude, longitude and altitude in both unsigned int and float 
 * @return true/false : GPS position lock status 
 */
bool VehicleHardware::GPSGet(VehicleNavigation::Location &location)
{
    // 
}

//=======================================================================================


//=======================================================================================
// IMU 

/**
 * @brief Read from IMU device(s) 
 * 
 * @details This function is called periodically from the autopilots communication thread 
 *          to read new data from an IMU device. Data is read here but not retreived by 
 *          the autopilot. Instead, the data_ready.imu_ready flag should be set if new 
 *          data is available and the autopilot will call IMUGet to collect the data 
 *          during the main thread. 
 * 
 * @see IMUGet 
 */
void VehicleHardware::IMURead(void)
{
    // 
}


/**
 * @brief Get the most recent IMU data 
 * 
 * @details This function is called from the autopilots main thread to collect new data 
 *          read from an IMU device if it's available. Data is copied here but not read. 
 *          This function is only called if the data_ready.imu_ready flag is set in the 
 *          IMURead function. 
 * 
 * @see IMURead 
 * 
 * @param accel : accelerometer data 
 * @param gyro : gyroscope data 
 * @param heading : compass/magnetometer heading (degrees*10) 
 */
void VehicleHardware::IMUGet(
    VehicleNavigation::Vector<int16_t> &accel, 
    VehicleNavigation::Vector<int16_t> &gyro, 
    int16_t &heading)
{
    // 
}

//=======================================================================================


//=======================================================================================
// Memory 

/**
 * @brief Read from a memory storage device 
 */
void VehicleHardware::MemoryRead(void)
{
    // 
}


/**
 * @brief Write to a memory storage device 
 */
void VehicleHardware::MemoryWrite(void)
{
    // 
}

//=======================================================================================


//=======================================================================================
// RC 

/**
 * @brief Read from an RC receiver 
 * 
 * @details This function is called periodically from the autopilots communication thread 
 *          to read new data from an RC receiver. Data is read here but not retreived by 
 *          the autopilot. Instead, the data_ready.rc_ready flag should be set if new 
 *          data is available and the autopilot will call RCGet to collect the data 
 *          during the main thread. 
 * 
 * @see RCGet 
 */
void VehicleHardware::RCRead(void)
{
    // 
}


/**
 * @brief Get data read from an RC receiver 
 * 
 * @details This function is called from the autopilots main thread to collect new data 
 *          read from an RC receiver if it's available. Data is copied here but not read. 
 *          This function is only called if the data_ready.rc_ready flag is set in the 
 *          RCRead function. 
 * 
 * @see RCRead 
 * 
 * @param channels : RC channels to populate 
 */
void VehicleHardware::RCGet(VehicleControl::ChannelFunctions &channels)
{
    // 
}

//=======================================================================================


//=======================================================================================
// Telemetry 

/**
 * @brief Read from a telemetry device 
 * 
 * @details This function is called periodically from the autopilots communication thread 
 *          to read new data from a telemetry device. Data is read here but not retreived 
 *          by the autopilot. Instead, the data_ready.telemetry_ready flag should be set 
 *          if new data is available and the autopilot will call TelemetryGet to collect 
 *          the data during the main thread. 
 * 
 * @see TelemetryGet 
 */
void VehicleHardware::TelemetryRead(void)
{
    // 
}


/**
 * @brief Get data read from a telemetry device 
 * 
 * @details This function is called from the autopilots main thread to collect new data 
 *          read from a telemetry device if it's available. Data is copied here but not 
 *          read. This function is only called if the data_ready.telemetry_ready flag is 
 *          set in the TelemetryRead function. 
 * 
 * @see TelemetryRead 
 * 
 * @param size : size of data read 
 * @param buffer : buffer to store new data 
 */
void VehicleHardware::TelemetryGet(uint16_t &size, uint8_t *buffer)
{
    // 
}


/**
 * @brief Set data to write to a telemtry device 
 * 
 * @details This function is called from the autopilots main thread to set data to write 
 *          to a telemetry device if there's new data to be sent. Data is copied here but 
 *          not written. If this function is called then TelemetryWrite will be executed 
 *          shortly afterwards. 
 * 
 * @see TelemetryWrite 
 * 
 * @param size : size of data to write 
 * @param buffer : buffer containing the data to write 
 */
void VehicleHardware::TelemetrySet(uint16_t &size, uint8_t *buffer)
{
    // 
}


/**
 * @brief Write to a telemetry device 
 * 
 * @details This function is called after the TelemetrySet function from the autopilots 
 *          communications thread to write data to a telemetry device. Data is written 
 *          here but not set. 
 * 
 * @see TelemetrySet 
 */
void VehicleHardware::TelemetryWrite(void)
{
    // 
}

//=======================================================================================
