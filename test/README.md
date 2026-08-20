# Tests

The test suite will be divided into:

- Unit tests for the EKF.
- Unit tests for visual odometry.
- IMU calibration tests.
- Mapper tests.
- ROS 2 integration tests.
- Tests using recorded sensor data.

Tests must not require CSI camera hardware, SPI devices, an ADIS16460, or
external network access. Hardware-specific tests will be executed separately
on the Raspberry Pi.
