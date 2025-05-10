#pragma once
#include <SoftwareWire.h>
#include "I2CInterface.h"

class SoftwareWireWrapper : public I2CInterface
{
public:
  SoftwareWireWrapper(SoftwareWire &w) : softWire(w) {}
  void begin() override { softWire.begin(); }
  void beginTransmission(uint8_t address) override { softWire.beginTransmission(address); }
  size_t write(uint8_t data) override { return softWire.write(data); }
  uint8_t endTransmission() override { return softWire.endTransmission(); }
  size_t requestFrom(uint8_t address, size_t quantity) override { return softWire.requestFrom(address, quantity); }
  int available() override { return softWire.available(); }
  int read() override { return softWire.read(); }

private:
  SoftwareWire &softWire;
};
