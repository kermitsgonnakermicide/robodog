#pragma once
#include <pigpio.h>
#include <cmath>

class PCA9685 {
public:
  explicit PCA9685(int i2c_addr);
  ~PCA9685();
  void setPWMFreq(int freq);
  void setPWM(int channel, int on, int off);
  void setAngle(int channel, float angle); // radians

private:
  int handle_;
  int i2c_addr_;
};
