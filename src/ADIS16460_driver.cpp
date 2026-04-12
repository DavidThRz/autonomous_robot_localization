
#include "autonomous_robot_localization/ADIS16460_driver.hpp"

ADIS16460_driver::ADIS16460_driver(const std::string& spi_device)
{
    spi_device_ = spi_device;
    error_state = false;

    isp_fd = open(spi_device_.c_str(), O_RDWR);
    if (isp_fd < 0) 
    {
        error_state = true;
        throw std::runtime_error("Error: opening " + spi_device_);
    }

    uint8_t spi_mode = SPI_MODE_3; /* ADIS16460 uses CPOL=1 CPHA=1 */
    if (ioctl(isp_fd, SPI_IOC_WR_MODE, &spi_mode) < 0) 
    {
        error_state = true;
        throw std::runtime_error("Error: setting SPI mode.");
    }

    imu_bits_per_word = 8;
    if (ioctl(isp_fd, SPI_IOC_WR_BITS_PER_WORD, &imu_bits_per_word) < 0) 
    {
        error_state = true;
        throw std::runtime_error("Error: setting bits per word.");
    }

    imu_frequency_hz = 1000000;
    if (ioctl(isp_fd, SPI_IOC_WR_MAX_SPEED_HZ, &imu_frequency_hz) < 0) 
    {
        error_state = true;
        throw std::runtime_error("Error: setting max speed.");
    }
}

ADIS16460_driver::~ADIS16460_driver()
{
    if (isp_fd >= 0) {
        close(isp_fd);
    }
}

int ADIS16460_driver::readRegister(uint8_t reg, uint16_t &value)
{
    if (error_state)
        return -1;

    if (sendSPI(reg) < 0)
        return -1;
    usleep(kSpiStallTimeUs);
    if (sendSPI(0x00, 0x00, &value) < 0)
        return -1;

    return 0;
}

int ADIS16460_driver::writeRegister(uint8_t reg, uint8_t value)
{
    if (error_state)
        return -1;

    reg |= 0x80;
    if (sendSPI(reg, value) < 0)
        return -1;

    return 0;
}

int ADIS16460_driver::writeRegister(uint8_t reg, uint16_t value)
{
    if (error_state)
        return -1;

    reg |= 0x80;
    if (sendSPI(reg, value & 0xFF) < 0)
        return -1;
    if (sendSPI(reg + 1, (value >> 8) & 0xFF) < 0)
        return -1;

    return 0;
}

int ADIS16460_driver::sendSPI(uint8_t reg, uint8_t value, uint16_t* response)
{
    if (error_state)
        return -1;

    std::lock_guard<std::mutex> lock(spi_mutex_);

    uint8_t tx_buf[2] = {reg, value};
    uint8_t rx_resp[2] = {0, 0};

    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx_buf;
    tr.rx_buf = (unsigned long)rx_resp;
    tr.len = 2;
    tr.speed_hz = imu_frequency_hz;
    tr.bits_per_word = imu_bits_per_word;

    if (ioctl(isp_fd, SPI_IOC_MESSAGE(1), &tr) < 1) 
    {
        throw std::runtime_error("Error: SPI communication failed.");
    }

    if (response) 
        *response = (rx_resp[0] << 8) | rx_resp[1];

    return 0;
}

bool ADIS16460_driver::testImu()
{
    uint16_t value;
    if (readRegister(PROD_ID, value) < 0) 
        return false;

    if (value != 0x404C) 
        return false;

    return true;
}

bool ADIS16460_driver::getIMUData(double &gyro_x, double &gyro_y, double &gyro_z, double &accl_x, double &accl_y, double &accl_z)
{
    constexpr double GYRO_SCALE = 0.005 * kPi / 180.0;  /* rad/s per LSB */
    constexpr double ACCL_SCALE = 0.25 * kGravity / 1000.0;   /* mg per LSB */

    uint16_t raw_x, raw_y, raw_z;
    if (readRegister(X_GYRO_OUT, raw_x) < 0) return false;
    if (readRegister(Y_GYRO_OUT, raw_y) < 0) return false;
    if (readRegister(Z_GYRO_OUT, raw_z) < 0) return false;

    gyro_x = static_cast<int16_t>(raw_x) * GYRO_SCALE;
    gyro_y = static_cast<int16_t>(raw_y) * GYRO_SCALE;
    gyro_z = static_cast<int16_t>(raw_z) * GYRO_SCALE;

    if (readRegister(X_ACCL_OUT, raw_x) < 0) return false;
    if (readRegister(Y_ACCL_OUT, raw_y) < 0) return false;
    if (readRegister(Z_ACCL_OUT, raw_z) < 0) return false;

    accl_x = static_cast<int16_t>(raw_x) * ACCL_SCALE;
    accl_y = static_cast<int16_t>(raw_y) * ACCL_SCALE;
    accl_z = static_cast<int16_t>(raw_z) * ACCL_SCALE;

    return true;
}

bool ADIS16460_driver::setBiasOffsets(double gyro_x_offset, double gyro_y_offset, double gyro_z_offset, double accl_x_offset, double accl_y_offset, double accl_z_offset)
{
    if (error_state)
        return false;

    constexpr double GYRO_OFFSET_SCALE = 0.000625 * kPi / 180.0;  /* rad/s per LSB */
    constexpr double ACCL_OFFSET_SCALE = 0.03125 * kGravity / 1000.0;   /* mg per LSB */

    uint16_t gyro_x_reg = static_cast<int>(gyro_x_offset / GYRO_OFFSET_SCALE);
    uint16_t gyro_y_reg = static_cast<int>(gyro_y_offset / GYRO_OFFSET_SCALE);
    uint16_t gyro_z_reg = static_cast<int>(gyro_z_offset / GYRO_OFFSET_SCALE);

    if (writeRegister(X_GYRO_OFF, gyro_x_reg) < 0) return false;
    if (writeRegister(Y_GYRO_OFF, gyro_y_reg) < 0) return false;
    if (writeRegister(Z_GYRO_OFF, gyro_z_reg) < 0) return false;

    uint16_t accl_x_reg = static_cast<int>(accl_x_offset / ACCL_OFFSET_SCALE);
    uint16_t accl_y_reg = static_cast<int>(accl_y_offset / ACCL_OFFSET_SCALE);
    uint16_t accl_z_reg = static_cast<int>(accl_z_offset / ACCL_OFFSET_SCALE);

    if (writeRegister(X_ACCL_OFF, accl_x_reg) < 0) return false;
    if (writeRegister(Y_ACCL_OFF, accl_y_reg) < 0) return false;
    if (writeRegister(Z_ACCL_OFF, accl_z_reg) < 0) return false;

    return true;
}

bool ADIS16460_driver::getBiasOffsets(double& gyro_x_offset, double& gyro_y_offset, double& gyro_z_offset, double& accl_x_offset, double& accl_y_offset, double& accl_z_offset)
{
    if (error_state)
        return false;

    constexpr double GYRO_SCALE = 0.000625 * kPi / 180.0;  /* rad/s per LSB */
    constexpr double ACCL_SCALE = 0.03125 * kGravity / 1000.0;   /* mg per LSB */

    uint16_t gyro_x_reg, gyro_y_reg, gyro_z_reg;
    if (readRegister(X_GYRO_OFF, gyro_x_reg) < 0) return false;
    if (readRegister(Y_GYRO_OFF, gyro_y_reg) < 0) return false;
    if (readRegister(Z_GYRO_OFF, gyro_z_reg) < 0) return false;

    gyro_x_offset = static_cast<int16_t>(gyro_x_reg) * GYRO_SCALE;
    gyro_y_offset = static_cast<int16_t>(gyro_y_reg) * GYRO_SCALE;
    gyro_z_offset = static_cast<int16_t>(gyro_z_reg) * GYRO_SCALE;

    uint16_t accl_x_reg, accl_y_reg, accl_z_reg;
    if (readRegister(X_ACCL_OFF, accl_x_reg) < 0) return false;
    if (readRegister(Y_ACCL_OFF, accl_y_reg) < 0) return false;
    if (readRegister(Z_ACCL_OFF, accl_z_reg) < 0) return false;

    accl_x_offset = static_cast<int16_t>(accl_x_reg) * ACCL_SCALE;
    accl_y_offset = static_cast<int16_t>(accl_y_reg) * ACCL_SCALE;
    accl_z_offset = static_cast<int16_t>(accl_z_reg) * ACCL_SCALE;

    return true;
}

