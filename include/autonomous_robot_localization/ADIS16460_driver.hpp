
#pragma once

#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <mutex>

class ADIS16460_driver
{
public:
    ADIS16460_driver(const std::string& spi_device);
    ~ADIS16460_driver();

    /* @brief Tests if IMU is functioning correctly checking its product identifier */
    bool testImu();

    /* @brief Reads gyroscope and accelerometer data */
    bool getIMUData(double &gyro_x, double &gyro_y, double &gyro_z, double &accl_x, double &accl_y, double &accl_z);

    /* @brief Sets the bias offset values for each axis */
    bool setBiasOffsets(double gyro_x_offset, double gyro_y_offset, double gyro_z_offset, double accl_x_offset, double accl_y_offset, double accl_z_offset);

    /* @brief Gets the current bias offset values for each axis */
    bool getBiasOffsets(double& gyro_x_offset, double& gyro_y_offset, double& gyro_z_offset, double& accl_x_offset, double& accl_y_offset, double& accl_z_offset);

    /* Physical constants */
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kGravity = 9.80665;  /* m/s^2 */

    /* SPI timing — ADIS16460 datasheet Table 2: t_STALL */
    static constexpr int kSpiStallTimeUs = 16;

private:

    int readRegister(uint8_t reg, uint16_t &value);
    int writeRegister(uint8_t reg, uint8_t value);
    int writeRegister(uint8_t reg, uint16_t value);

    int sendSPI(uint8_t reg, uint8_t value = 0x00, uint16_t* response = nullptr);

    std::string spi_device_;
    int isp_fd;

    bool error_state;

    uint32_t imu_frequency_hz;
    uint8_t imu_bits_per_word;

    std::mutex spi_mutex_;

    /* ADIS16460 Register Map */
    static constexpr uint8_t FLASH_CNT  = 0x00;  /* Flash memory write count */
    static constexpr uint8_t DIAG_STAT  = 0x02;  /* Diagnostic and operational status */

    static constexpr uint8_t X_GYRO_LOW = 0x04;  /* X-axis gyroscope output, lower word */
    static constexpr uint8_t X_GYRO_OUT = 0x06;  /* X-axis gyroscope output, upper word */
    static constexpr uint8_t Y_GYRO_LOW = 0x08;  /* Y-axis gyroscope output, lower word */
    static constexpr uint8_t Y_GYRO_OUT = 0x0A;  /* Y-axis gyroscope output, upper word */
    static constexpr uint8_t Z_GYRO_LOW = 0x0C;  /* Z-axis gyroscope output, lower word */
    static constexpr uint8_t Z_GYRO_OUT = 0x0E;  /* Z-axis gyroscope output, upper word */

    static constexpr uint8_t X_ACCL_LOW = 0x10;  /* X-axis accelerometer output, lower word */
    static constexpr uint8_t X_ACCL_OUT = 0x12;  /* X-axis accelerometer output, upper word */
    static constexpr uint8_t Y_ACCL_LOW = 0x14;  /* Y-axis accelerometer output, lower word */
    static constexpr uint8_t Y_ACCL_OUT = 0x16;  /* Y-axis accelerometer output, upper word */
    static constexpr uint8_t Z_ACCL_LOW = 0x18;  /* Z-axis accelerometer output, lower word */
    static constexpr uint8_t Z_ACCL_OUT = 0x1A;  /* Z-axis accelerometer output, upper word */

    static constexpr uint8_t SMPL_CNTR  = 0x1C;  /* Sample counter, MSC_CTRL[3:2] = 11 */
    static constexpr uint8_t TEMP_OUT   = 0x1E;  /* Temperature (internal, not calibrated) */

    static constexpr uint8_t X_DELT_ANG = 0x24;  /* X-axis delta angle output */
    static constexpr uint8_t Y_DELT_ANG = 0x26;  /* Y-axis delta angle output */
    static constexpr uint8_t Z_DELT_ANG = 0x28;  /* Z-axis delta angle output */
    static constexpr uint8_t X_DELT_VEL = 0x2A;  /* X-axis delta velocity output */
    static constexpr uint8_t Y_DELT_VEL = 0x2C;  /* Y-axis delta velocity output */
    static constexpr uint8_t Z_DELT_VEL = 0x2E;  /* Z-axis delta velocity output */

    static constexpr uint8_t MSC_CTRL   = 0x32;  /* Miscellaneous control */
    static constexpr uint8_t SYNC_SCAL  = 0x34;  /* Sync input scale control */
    static constexpr uint8_t DEC_RATE   = 0x36;  /* Decimation rate control */
    static constexpr uint8_t FLTR_CTRL  = 0x38;  /* Filter control, autonull record time */

    static constexpr uint8_t GLOB_CMD   = 0x3E;  /* Global commands */

    static constexpr uint8_t X_GYRO_OFF = 0x40;  /* X-axis gyroscope bias offset factor */
    static constexpr uint8_t Y_GYRO_OFF = 0x42;  /* Y-axis gyroscope bias offset factor */
    static constexpr uint8_t Z_GYRO_OFF = 0x44;  /* Z-axis gyroscope bias offset factor */
    static constexpr uint8_t X_ACCL_OFF = 0x46;  /* X-axis accelerometer bias offset factor */
    static constexpr uint8_t Y_ACCL_OFF = 0x48;  /* Y-axis accelerometer bias offset factor */
    static constexpr uint8_t Z_ACCL_OFF = 0x4A;  /* Z-axis accelerometer bias offset factor */

    static constexpr uint8_t LOT_ID1    = 0x52;  /* Lot Identification Number 1 */
    static constexpr uint8_t LOT_ID2    = 0x54;  /* Lot Identification Number 2 */
    static constexpr uint8_t PROD_ID    = 0x56;  /* Product identifier */
    static constexpr uint8_t SERIAL_NUM = 0x58;  /* Lot serial number */
    static constexpr uint8_t CAL_SGNTR  = 0x60;  /* Calibration memory signature value */
    static constexpr uint8_t CAL_CRC    = 0x62;  /* Calibration memory CRC values */
    static constexpr uint8_t CODE_SGNTR = 0x64;  /* Code memory signature value */
    static constexpr uint8_t CODE_CRC   = 0x66;  /* Code memory CRC values */
};
