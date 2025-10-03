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
// Debug 

/**
 * @brief Write data to a serial connection 
 * 
 * @details This function is used when the user needs to temporarily send data to an 
 *          external device for things such as debugging or IMU calibration through 
 *          3rd party software. VH_DEBUG_OUTPUT must be enabled in the system config 
 *          for this function to be called. This function will be called regardless of 
 *          vehicle state. 
 */
void VehicleHardware::DebugWrite(void)
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
 * @param location : buffer to store latitude, longitude and altitude from the GPS 
 * @param location_uncertainty : buffer to store location uncertainty from the GPS measurement 
 * @param velocity : buffer to store speed over ground, course over ground and vertical velocity from the GPS 
 * @param velocity_uncertainty : buffer to store velocity uncertainty from the GPS measurement 
 * @return true/false : GPS position lock status 
 */
bool VehicleHardware::GPSGet(
    VehicleNavigation::Location &location,
    VehicleNavigation::Location &location_uncertainty,
    VehicleNavigation::Velocity &velocity,
    VehicleNavigation::Velocity &velocity_uncertainty)
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
 *          IMUs should have their positive X-axis in the vehicles forward direction, 
 *          their positive y-axis pointing to the left and the positive z-axis pointing 
 *          up (NWU orientation following right hand rule) to ensure orientation is 
 *          calculated properly. If the accelerometer, gyroscope or magnetometer axes 
 *          don't align with this then just invert the sign of the axis reading. If signs 
 *          are inverted, make sure calibration values accommodate this. 
 *          
 *          Magnetometer readings can be supplied as calibrated or uncalibrated values. 
 *          If not pre-calibrated, make sure to enable calibration correction and set 
 *          the magnetometer calibration parameters (hard and soft iron offsets). See 
 *          VS_MAG_CAL in the system configuration file. 
 *          
 *          Note that the accelerometer and gyroscope units matter but the magnetometer 
 *          units don't. Acceleration should be provided in g's and angular rate should 
 *          be provided in deg/s. Calculations that use magnetic fiels only care about 
 *          the magnitude between axes. However, typical magnetometer units are mG or uT. 
 * 
 * @see IMURead 
 * 
 * @param accel : 3-axis accelerometer data (g's)
 * @param gyro : 3-axis gyroscope data (deg/s) 
 * @param mag : 3-axis magnetometer axis data (typically mG or uT) 
 */
void VehicleHardware::IMUGet(
    VehicleNavigation::Vector<float> &accel, 
    VehicleNavigation::Vector<float> &gyro, 
    VehicleNavigation::Vector<float> &mag)
{
    // 
}

//=======================================================================================


//=======================================================================================
// Memory 

/**
 * @brief Setup the external memory device for file access 
 * 
 * @details The autopilot calls this function to establish the external memory device. 
 *          Everything needed to get talking to the external memory should be done here. 
 *          This could include, but is not limited to, mounting the device, checking for 
 *          sufficient free space, or creating the directory from which to operate from. 
 *          See the description of the MemorySetFileName function for options on file 
 *          paths and directories. No files should be created or opened here as that is 
 *          handled through the implementation of the other memory functions. By the 
 *          end of this function, the device should be ready for the autopilot to use. 
 *          
 *          Note that the autopilot makes decisions based on the status of these functions 
 *          so it's important the user provides the correct return values. 
 * 
 * @see MemorySetFileName
 * 
 * @return VehicleHardware::MemoryStatus : MEMORY_CAP_ERROR --> Insufficient free space on device 
 *                                         MEMORY_ACCESS_ERROR --> Problem accessing device 
 *                                         MEMORY_OK --> Memory ready for access, everything OK 
 */
VehicleHardware::MemoryStatus VehicleHardware::MemorySetup(void)
{
    return MemoryStatus::MEMORY_OK;
}


/**
 * @brief Set the name of the file to access 
 * 
 * @details The autopilot will call this function when it needs to specify a file to 
 *          open. The file name is provided and this function must use the name to 
 *          establish a path to the file. That path will then be used to open the file 
 *          when the autopilot calls the MemoryOpenFile function. The user can handle the 
 *          path in one of two ways: 
 *          1. The external memory device is already in the directory where files should 
 *             be accessed so only the file name is used as the path. Changing the 
 *             directory should be done in the MemorySetup function, NOT here. 
 *          2. Alternatively, the file name can be appended to the path of the directory 
 *             where files should be accessed. This way the full path from the root is 
 *             used to open the file. 
 *          
 *          The buffer used to store the path should be large enough to store whatever 
 *          path method the user chooses. 
 * 
 * @see MemoryOpenFile
 * @see MemorySetup
 * 
 * @param file_name : string containing the name of the file to be opened 
 * @param file_name_size : size of the file name 
 */
void VehicleHardware::MemorySetFileName(
    const char *file_name, 
    uint16_t file_name_size)
{
    // 
}


/**
 * @brief Open a file on the external memory device 
 * 
 * @details This function will be called by the autopilot when a file needs to be opened 
 *          for reading or writing. The file name is set in MemorySetFileName which will 
 *          be called by the autopilot to establish a path before attempting to open the 
 *          file. This function must use the set path to open the file if it exists 
 *          (without overwritting it) or create and open the file if it does not exist. 
 *          The file should be opened with both read and write permissions and the 
 *          position in the file should be set to the start. The return status must 
 *          indicate if the file previously existed or not so the autopilot knows how to 
 *          handle the file. 
 *          
 *          Note that the autopilot makes decisions based on the status of these functions 
 *          so it's important the user provides the correct return values. 
 * 
 * @see MemorySetFileName
 * 
 * @return VehicleHardware::MemoryStatus : MEMORY_FILE_OPENED --> File exists and was opened 
 *                                         MEMORY_FILE_CREATED --> File did not exist, was created and opened 
 *                                         MEMORY_ACCESS_ERROR --> Problem accessing device 
 */
VehicleHardware::MemoryStatus VehicleHardware::MemoryOpenFile(void)
{
    return MemoryStatus::MEMORY_FILE_OPENED;
}


/**
 * @brief Close a file on the external memory device 
 * 
 * @details This function will be called by the autopilot when an open file needs to be 
 *          closed. The status of the close operation must be returned. 
 *          
 *          Note that the autopilot makes decisions based on the status of these functions 
 *          so it's important the user provides the correct return values. 
 * 
 * @return VehicleHardware::MemoryStatus : MEMORY_ACCESS_ERROR --> Problem accessing device 
 *                                         MEMORY_OK --> File closed, everything OK 
 */
VehicleHardware::MemoryStatus VehicleHardware::MemoryCloseFile(void)
{
    return MemoryStatus::MEMORY_OK;
}


/**
 * @brief Read data from the external memory device 
 * 
 * @details This function will be called by the autopilot when data needs to be read from 
 *          a file. The file that requires reading from should have already been opened 
 *          using the MemoryOpenFile function which the autopilot will have called before 
 *          attempting to read any data. This function must read one line of data from 
 *          the open file, save it and return the status of the operation. Saved data is 
 *          retrieved by the autopilot using the MemoryGetData function which will be 
 *          called following the call to this function. Data in files are formatted into 
 *          individual lines of information and each line gets read one at a time through 
 *          repeated calls to this function. The buffer that read data is saved to must 
 *          be at least the size of 'memory_buff_size' so that all data from a line can 
 *          be captured. 
 *          
 *          Note that the autopilot makes decisions based on the status of these functions 
 *          so it's important the user provides the correct return values. 
 * 
 * @see MemoryGetData
 * 
 * @return VehicleHardware::MemoryStatus : MEMORY_EOF --> End of file reached (no more data) 
 *                                         MEMORY_ACCESS_ERROR --> Problem accessing device 
 *                                         MEMORY_OK --> Data read, everything OK 
 */
VehicleHardware::MemoryStatus VehicleHardware::MemoryRead(void)
{
    return MemoryStatus::MEMORY_OK;
}


/**
 * @brief Get the data read from external memory 
 * 
 * @details The autopilot will call this function to fetch new data that was read from 
 *          external memory device. Data that was saved in the MemoryRead function should 
 *          be copied to the provided buffer so the autopilot can use it. Data is 
 *          formatted into individual lines of information and each line gets read one 
 *          at a time through repeated calls to the read and get functions.
 * 
 * @see MemoryRead
 * 
 * @param data_buff : buffer to store data read from the device 
 * @param data_buff_size : size of buffer that will store the data 
 */
void VehicleHardware::MemoryGetData(
    char *data_buff, 
    uint16_t data_buff_size)
{
    // 
}


/**
 * @brief Set the data to be written to external memory 
 * 
 * @details The autopilot will call this function to stage new data to be writing to the 
 *          external memory device. The provided data should be copied so that it can be 
 *          written when the MemoryWrite function is called. The buffer that the data is 
 *          copied to must be large enough to hold all the provided data. Data is 
 *          formatted into individual lines of information and each line gets written one 
 *          at a time through repeated calls to the set and write functions.
 * 
 * @see MemoryWrite
 * 
 * @param data_buff : buffer containing data to write to the device 
 * @param data_buff_size : size of data in the buffer 
 */
void VehicleHardware::MemorySetData(
    const char *data_buff, 
    uint16_t data_buff_size)
{
    // 
}


/**
 * @brief Write data to the external memory device 
 * 
 * @details This function will be called by the autopilot when data needs to be written 
 *          to a file. The data to be written should have already been copied/saved in 
 *          the MemorySetData function which the autopilot will have called before this. 
 *          The file that requires writing to should have already been opened using the 
 *          MemoryOpenFile function which the autopilot will have also called before 
 *          attempting to write any data. This function must write the data to the open 
 *          file and return the status of the operation. Data is formatted into individual 
 *          lines of information and each line gets written one at a time through repeated 
 *          calls to the set and write functions. Each new line of data must be appended 
 *          to the end of the file. 
 *          
 *          Note that the autopilot makes decisions based on the status of these functions 
 *          so it's important the user provides the correct return values. 
 * 
 * @see MemorySetData
 * 
 * @return VehicleHardware::MemoryStatus : MEMORY_ACCESS_ERROR --> Problem accessing device 
 *                                         MEMORY_OK --> Data written, everything OK 
 */
VehicleHardware::MemoryStatus VehicleHardware::MemoryWrite(void)
{
    return MemoryStatus::MEMORY_OK;
}


/**
 * @brief Truncate data in an open file 
 * 
 * @details The autopilot will call this function when old file data needs to be removed. 
 *          In some cases, the autopilot needs to overwrite old data saved in external 
 *          memory, and truncating the old data ensures no old data will be left behind 
 *          once new data is done being written. This function should remove all the 
 *          existing file data from the current position within the open file until the 
 *          end of the file. This function shouldn't navigate to a certain point within 
 *          the file as the autopilot will call this function when it needs to. 
 *          
 *          Note that the autopilot makes decisions based on the status of these functions 
 *          so it's important the user provides the correct return values. 
 * 
 * @return VehicleHardware::MemoryStatus : MEMORY_ACCESS_ERROR --> Problem accessing device 
 *                                         MEMORY_OK --> Data truncated, everything OK 
 */
VehicleHardware::MemoryStatus VehicleHardware::MemoryTruncate(void)
{
    return MemoryStatus::MEMORY_OK;
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
 * @param size : buffer to store the size of data read 
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
