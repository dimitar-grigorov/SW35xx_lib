#pragma once
#include <stdint.h>

class I2CInterface {
public:
  virtual void begin() = 0;
  virtual void beginTransmission(uint8_t address) = 0;
  virtual size_t write(uint8_t data) = 0;
  virtual uint8_t endTransmission() = 0;
  virtual size_t requestFrom(uint8_t address, size_t quantity) = 0;
  virtual int available() = 0;
  virtual int read() = 0;
};
