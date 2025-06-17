#include "servo_driver/pigpio_pwm_controller.hpp"
#include <stdexcept>
#include <unistd.h>

PCA9685::PCA9685(int i2c_addr) : i2c_addr_(i2c_addr) {
  if (gpioInitialise() < 0) {
    throw std::runtime_error("pigpio init failed");
  }

  handle_ = i2cOpen(1, i2c_addr_, 0);
  if (handle_ < 0) {
    throw std::runtime_error("Failed to open I2C");
  }

  i2cWriteByteData(handle_, 0x00, 0x00); // MODE1 reset
  setPWMFreq(50);
}

PCA9685::~PCA9685() {
  i2cClose(handle_);
  gpioTerminate();
}

void PCA9685::setPWMFreq(int freq) {
  float prescaleval = 25000000.0 / (4096.0 * freq) - 1.0;
  uint8_t prescale = static_cast<uint8_t>(prescaleval + 0.5);

  uint8_t oldmode = i2cReadByteData(handle_, 0x00);
  uint8_t newmode = (oldmode & 0x7F) | 0x10;

  i2cWriteByteData(handle_, 0x00, newmode);
  i2cWriteByteData(handle_, 0xFE, prescale);
  i2cWriteByteData(handle_, 0x00, oldmode);
  usleep(5000);
  i2cWriteByteData(handle_, 0x00, oldmode | 0xA1);
}

void PCA9685::setPWM(int channel, int on, int off) {
  i2cWriteByteData(handle_, 0x06 + 4 * channel, on & 0xFF);
  i2cWriteByteData(handle_, 0x07 + 4 * channel, on >> 8);
  i2cWriteByteData(handle_, 0x08 + 4 * channel, off & 0xFF);
  i2cWriteByteData(handle_, 0x09 + 4 * channel, off >> 8);
}

void PCA9685::setAngle(int channel, float angle) {
  float deg = angle * 180.0 / M_PI;
  float pulse = 1500 + (deg / 90.0) * 1000;
  int ticks = static_cast<int>(pulse / 20000.0 * 4096);
  setPWM(channel, 0, ticks);
}
