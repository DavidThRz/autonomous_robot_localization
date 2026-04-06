
#include "autonomous_robot_localization/ADIS16460_driver.hpp"

ADIS16460_driver::ADIS16460_driver() : isp_device("/dev/spidev0.0")
{
    error_state = false;

    isp_fd = open(isp_device, O_RDWR);
    if (isp_fd < 0) 
    {
        error_state = true;
        std::cerr << "Error: opening " << isp_device << ".\n";
        return;
    }

    uint8_t spi_mode = SPI_MODE_3; /* ADIS16460 uses CPOL=1 CPHA=1 */
    if (ioctl(isp_fd, SPI_IOC_WR_MODE, &spi_mode) < 0) 
    {
        error_state = true;
        std::cerr << "Error: setting SPI mode.\n";
        return;
    }

    imu_bits_per_word = 8;
    if (ioctl(isp_fd, SPI_IOC_WR_BITS_PER_WORD, &imu_bits_per_word) < 0) 
    {
        error_state = true;
        std::cerr << "Error: setting bits per word.\n";
        return;
    }

    imu_frequency_hz = 1000000;
    if (ioctl(isp_fd, SPI_IOC_WR_MAX_SPEED_HZ, &imu_frequency_hz) < 0) 
    {
        error_state = true;
        std::cerr << "Error: setting max speed.\n";
        return;
    }
}

ADIS16460_driver::~ADIS16460_driver()
{
    if (!error_state) {
        close(isp_fd);
    }
}

int ADIS16460_driver::readRegister(uint8_t reg, uint16_t &value)
{
    if (error_state)
        return -1;

    sendSPI(reg);
    usleep(16);
    sendSPI(0x00, 0x00, &value);

    return 0;
}

int ADIS16460_driver::writeRegister(uint8_t reg, uint8_t value)
{
    if (error_state)
        return -1;

    reg |= 0x80;
    sendSPI(reg, value);

    return 0;
}

int ADIS16460_driver::writeRegister(uint8_t reg, uint16_t value)
{
    if (error_state)
        return -1;

    reg |= 0x80;
    sendSPI(reg, value & 0xFF);
    sendSPI(reg + 1, (value >> 8) & 0xFF);

    return 0;
}

int ADIS16460_driver::sendSPI(uint8_t reg, uint8_t value, uint16_t* response)
{
    if (error_state)
        return -1;

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
        std::cerr << "Error: SPI communication failed.\n";
        return -1;
    }

    if (response) 
        *response = (rx_resp[0] << 8) | rx_resp[1];

    return 0;
}

bool ADIS16460_driver::testImu()
{
    uint16_t value;
    if (readRegister(0x56, value) == 1) 
        return false;

    if (value != 0x404C) 
        return false;

    return true;
}

bool ADIS16460_driver::getIMUData(double &gyro_x, double &gyro_y, double &gyro_z, double &accl_x, double &accl_y, double &accl_z)
{
    constexpr double GYRO_SCALE = 0.005 * C_PI / 180.0;  /* rad/s per LSB */
    constexpr double ACCL_SCALE = 0.25 * C_g / 1000.0;   /* mg per LSB */

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

void ADIS16460_driver::setBiasOffsets(double gyro_x_offset, double gyro_y_offset, double gyro_z_offset, double accl_x_offset, double accl_y_offset, double accl_z_offset)
{
    if (error_state)
        return;

    constexpr double GYRO_OFFSET_SCALE = 0.000625 * C_PI / 180.0;  /* rad/s per LSB */
    constexpr double ACCL_OFFSET_SCALE = 0.03125 * C_g / 1000.0;   /* mg per LSB */

    uint16_t gyro_x_reg = static_cast<int>(gyro_x_offset / GYRO_OFFSET_SCALE);
    uint16_t gyro_y_reg = static_cast<int>(gyro_y_offset / GYRO_OFFSET_SCALE);
    uint16_t gyro_z_reg = static_cast<int>(gyro_z_offset / GYRO_OFFSET_SCALE);

    writeRegister(X_GYRO_OFF, gyro_x_reg);
    writeRegister(Y_GYRO_OFF, gyro_y_reg);
    writeRegister(Z_GYRO_OFF, gyro_z_reg);

    uint16_t accl_x_reg = static_cast<int>(accl_x_offset / ACCL_OFFSET_SCALE);
    uint16_t accl_y_reg = static_cast<int>(accl_y_offset / ACCL_OFFSET_SCALE);
    uint16_t accl_z_reg = static_cast<int>(accl_z_offset / ACCL_OFFSET_SCALE);

    writeRegister(X_ACCL_OFF, accl_x_reg);
    writeRegister(Y_ACCL_OFF, accl_y_reg);
    writeRegister(Z_ACCL_OFF, accl_z_reg);
}

void ADIS16460_driver::resetBiasOffsets()
{
    if (error_state)
        return;

    for (uint8_t reg = X_GYRO_OFF; reg <= Z_ACCL_OFF; reg += 2) 
    {
        if (writeRegister(reg, static_cast<uint16_t>(0x0000)) < 0) return;
    }
}

