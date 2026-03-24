
#include "autonomous_robot_localization/ADIS16460_driver.hpp"

int main() 
{
    ADIS16460_driver imu_driver;

    uint16_t value;
    if (imu_driver.readRegister(0x56, value) == 0) {
        std::cout << "Valor leido del registro: 0x" << std::hex << std::uppercase << value << std::endl;
    } else {
        std::cerr << "Error leyendo el registro." << std::endl;
    }

    return 0;
}