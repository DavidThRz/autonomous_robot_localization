
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
    
    uint8_t tx_req[2]   = {reg, 0x00};
    uint8_t rx_dummy[2] = {0, 0};

    uint8_t tx_dummy[2] = {0x00, 0x00};
    uint8_t rx_resp[2]  = {0, 0};

    struct spi_ioc_transfer tr_req;
    memset(&tr_req, 0, sizeof(tr_req));
    tr_req.tx_buf = (unsigned long)tx_req;
    tr_req.rx_buf = (unsigned long)rx_dummy;
    tr_req.len = 2;
    tr_req.speed_hz = imu_frequency_hz;
    tr_req.bits_per_word = imu_bits_per_word;

    if (ioctl(isp_fd, SPI_IOC_MESSAGE(1), &tr_req) < 1) 
    {
        std::cerr << "Error: sending request.\n";
        return -1;
    }

    usleep(16);

    struct spi_ioc_transfer tr_res;
    memset(&tr_res, 0, sizeof(tr_res));
    tr_res.tx_buf = (unsigned long)tx_dummy;
    tr_res.rx_buf = (unsigned long)rx_resp;
    tr_res.len = 2;
    tr_res.speed_hz = imu_frequency_hz;
    tr_res.bits_per_word = imu_bits_per_word;

    if (ioctl(isp_fd, SPI_IOC_MESSAGE(1), &tr_res) < 1) 
    {
        std::cerr << "Error: reading response.\n";
        return -1;
    }

    value = (rx_resp[0] << 8) | rx_resp[1];

    return 0;
}