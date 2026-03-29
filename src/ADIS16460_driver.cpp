
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
sendSPI(0x00, 0x00, &    value);

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

bool ADIS16460_driver::testGyroData()
{
    if (error_state)
        return false;

    uint16_t raw_x, raw_y, raw_z;
    if (readRegister(X_GYRO_OUT, raw_x) < 0) return false;
    if (readRegister(Y_GYRO_OUT, raw_y) < 0) return false;
    if (readRegister(Z_GYRO_OUT, raw_z) < 0) return false;

    double gyro_x, gyro_y, gyro_z;
    gyro_x = static_cast<int16_t>(raw_x) * 0.005;
    gyro_y = static_cast<int16_t>(raw_y) * 0.005;
    gyro_z = static_cast<int16_t>(raw_z) * 0.005;

    std::cout << "Gyroscope Data: " << std::endl;
    std::cout << "  GYRO_X: " << gyro_x << std::endl;
    std::cout << "  GYRO_Y: " << gyro_y << std::endl;
    std::cout << "  GYRO_Z: " << gyro_z << std::endl;

    return true;
}

bool ADIS16460_driver::testAcclData()
{
    if (error_state)
        return false;

    uint16_t raw_x, raw_y, raw_z;
    if (readRegister(X_ACCL_OUT, raw_x) < 0) return false;
    if (readRegister(Y_ACCL_OUT, raw_y) < 0) return false;
    if (readRegister(Z_ACCL_OUT, raw_z) < 0) return false;

    double accl_x, accl_y, accl_z;
    accl_x = static_cast<int16_t>(raw_x) * 0.25 * g_ / 1000.0;
    accl_y = static_cast<int16_t>(raw_y) * 0.25 * g_ / 1000.0;
    accl_z = static_cast<int16_t>(raw_z) * 0.25 * g_ / 1000.0;

    std::cout << "Accelerometer Data: " << std::endl;
    std::cout << "  ACCEL_X: " << accl_x << std::endl;
    std::cout << "  ACCEL_Y: " << accl_y << std::endl;
    std::cout << "  ACCEL_Z: " << accl_z << std::endl;

    return true;
}

void ADIS16460_driver::readOffsets()
{
    if (error_state)
        return;
    
    uint16_t x_gyro_offset, y_gyro_offset, z_gyro_offset;
    uint16_t x_accl_offset, y_accl_offset, z_accl_offset;
    
    if (readRegister(X_GYRO_OFF, x_gyro_offset) < 0) return;
    if (readRegister(Y_GYRO_OFF, y_gyro_offset) < 0) return;
    if (readRegister(Z_GYRO_OFF, z_gyro_offset) < 0) return;
    if (readRegister(X_ACCL_OFF, x_accl_offset) < 0) return;
    if (readRegister(Y_ACCL_OFF, y_accl_offset) < 0) return;
    if (readRegister(Z_ACCL_OFF, z_accl_offset) < 0) return;

    std::cout << "Gyroscope Offsets: " << std::endl;
    std::cout << "  X_GYRO_OFFSET: " << static_cast<int16_t>(x_gyro_offset) << std::endl;
    std::cout << "  Y_GYRO_OFFSET: " << static_cast<int16_t>(y_gyro_offset) << std::endl;
    std::cout << "  Z_GYRO_OFFSET: " << static_cast<int16_t>(z_gyro_offset) << std::endl;
    std::cout << "Accelerometer Offsets: " << std::endl;
    std::cout << "  X_ACCL_OFFSET: " << static_cast<int16_t>(x_accl_offset) << std::endl;
    std::cout << "  Y_ACCL_OFFSET: " << static_cast<int16_t>(y_accl_offset) << std::endl;
    std::cout << "  Z_ACCL_OFFSET: " << static_cast<int16_t>(z_accl_offset) << std::endl;
}

void ADIS16460_driver::readBiasEstimationTimeFactor()
{
    if (error_state)
        return;

    const uint16_t time_factor_mask = 0x0700;
    uint16_t raw_value;
    if (readRegister(FLTR_CTRL, raw_value) < 0) return;

    uint8_t bias_estimation_time_factor = (raw_value & time_factor_mask) >> 8;

    std::cout << "Bias Estimation Time Factor: " << static_cast<int>(bias_estimation_time_factor) << std::endl;
}

void ADIS16460_driver::setBiasEstimationTimeFactor(uint8_t time_factor)
{
    if (error_state)
        return;

    if (time_factor > 6) 
    {
        std::cerr << "Error: Time factor must be between 0 and 6" << std::endl;
        return;
    }

    if (writeRegister(0x39, time_factor) < 0) return;

    uint16_t raw_value;
    if (readRegister(FLTR_CTRL, raw_value) < 0) return;

    uint8_t changed_time_factor = (raw_value & 0x0700) >> 8;
    if (changed_time_factor != time_factor) 
    {
        std::cerr << "Error: Failed to set Bias Estimation Time Factor." << std::endl;
        return;
    }

    std::cout << "Bias estimation time factor successfully set to: " << static_cast<int>(time_factor) << std::endl;
}
