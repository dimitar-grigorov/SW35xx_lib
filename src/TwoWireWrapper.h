#pragma once
#include <Wire.h>
#include "I2CInterface.h"

class TwoWireWrapper : public I2CInterface {
public:
  TwoWireWrapper(TwoWire &w) : wire(w) {}
  void begin() override { wire.begin(); }
  void beginTransmission(uint8_t address) override { wire.beginTransmission(address); }
  size_t write(uint8_t data) override { return wire.write(data); }
  uint8_t endTransmission() override { return wire.endTransmission(); }
  size_t requestFrom(uint8_t address, size_t quantity) override { return wire.requestFrom(address, quantity); }
  int available() override { return wire.available(); }
  int read() override { return wire.read(); }
private:
  TwoWire &wire;
};
