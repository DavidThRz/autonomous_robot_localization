
#ifndef ADIS16460_HPP
#define ADIS16460_HPP

#pragma once

#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <iomanip>

class ADIS16460_driver
{
public:
    ADIS16460_driver();
    ~ADIS16460_driver();

    /* @brief Tests if IMU is functioning correctly checking its product identifier */
    bool testImu();

    /* @brief Outputs gyroscope data with a resolution of 0.005 º/s
     * @return true on success, false on failure
     */
    bool testGyroData();

    /* @brief Outputs accelerometer data with a resolution of 0.25 mg
     * @return true on success, false on failure
     */
    bool testAcclData();

    /* @brief Reads the offset values from the accelerometer registers */
    void readOffsets();

    /* @brief Reads register with time to estimate sensor bias for calibration */
    void readBiasEstimationTimeFactor();

    /* @brief Sets the bias estimation time factor in the filter control register */
    void setBiasEstimationTimeFactor(uint8_t time_factor);

private:

    int readRegister(uint8_t reg, uint16_t &value);
    int writeRegister(uint8_t reg, uint8_t value);

    int sendSPI(uint8_t reg, uint8_t value = 0x00, uint16_t* response = nullptr);

    const char* isp_device;
    int isp_fd;

    bool error_state;

    uint32_t imu_frequency_hz;
    uint8_t imu_bits_per_word;

};

#define g_         9.80665

/* ADIS16460 Registers */
#define FLASH_CNT  0x00  /* Flash memory write count */
#define DIAG_STAT  0x02  /* Diagnostic and operational status*/

#define X_GYRO_LOW 0x04  /* X-axis gyroscope output, lower word */
#define X_GYRO_OUT 0x06  /* X-axis gyroscope output, upper word */
#define Y_GYRO_LOW 0x08  /* Y-axis gyroscope output, lower word */
#define Y_GYRO_OUT 0x0A  /* Y-axis gyroscope output, upper word */
#define Z_GYRO_LOW 0x0C  /* Z-axis gyroscope output, lower word */
#define Z_GYRO_OUT 0x0E  /* Z-axis gyroscope output, upper word */

#define X_ACCL_LOW 0x10  /* X-axis accelerometer output, lower word */
#define X_ACCL_OUT 0x12  /* X-axis accelerometer output, upper word */
#define Y_ACCL_LOW 0x14  /* Y-axis accelerometer output, lower word */
#define Y_ACCL_OUT 0x16  /* Y-axis accelerometer output, upper word */
#define Z_ACCL_LOW 0x18  /* Z-axis accelerometer output, lower word */
#define Z_ACCL_OUT 0x1A  /* Z-axis accelerometer output, upper word */

#define SMPL_CNTR  0x1C  /* Sample counter, MSC_CTRL[3:2] = 11 */
#define TEMP_OUT   0x1E  /* Temperature (internal, not calibrated) */

#define X_DELT_ANG 0x24  /* X-axis delta angle output */
#define Y_DELT_ANG 0x26  /* Y-axis delta angle output */
#define Z_DELT_ANG 0x28  /* Z-axis delta angle output */
#define X_DELT_VEL 0x2A  /* X-axis delta velocity output */
#define Y_DELT_VEL 0x2C  /* Y-axis delta velocity output */
#define Z_DELT_VEL 0x2E  /* Z-axis delta velocity output */

#define MSC_CTRL   0x32  /* Miscellaneous control */
#define SYNC_SCAL  0x34  /* Sync input scale control */
#define DEC_RATE   0x36  /* Decimation rate control */
#define FLTR_CTRL  0x38  /* Filter control, autonull record time */

#define GLOB_CMD   0x3E  /* Global commands */

#define X_GYRO_OFF 0x40  /* X-axis gyroscope bias offset factor */
#define Y_GYRO_OFF 0x42  /* Y-axis gyroscope bias offset factor */
#define Z_GYRO_OFF 0x44  /* Z-axis gyroscope bias offset factor */
#define X_ACCL_OFF 0x46  /* X-axis accelerometer bias offset factor */
#define Y_ACCL_OFF 0x48  /* Y-axis accelerometer bias offset factor */
#define Z_ACCL_OFF 0x4A  /* Z-axis accelerometer bias offset factor */

#define LOT_ID1    0x52  /* Lot Identification Number 1 */
#define LOT_ID2    0x54  /* Lot Identification Number 2 */
#define PROD_ID    0x56  /* Product identifier */
#define SERIAL_NUM 0x58  /* Lot serial number */
#define CAL_SGNTR  0x60  /* Calibration memory signature value */
#define CAL_CRC    0x62  /* Calibration memory CRC values */
#define CODE_SGNTR 0x64  /* Code memory signature value */
#define CODE_CRC   0x66  /* Code memory CRC values */

#endif
