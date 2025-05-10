#include <Arduino.h>
#include <SoftwareWire.h>
#include <SW35xx_lib.h>
#include "TwoWireWrapper.h"
#include "SoftwareWireWrapper.h"

using namespace SW35xx_lib;

// 1) Hardware I²C
TwoWireWrapper i2cWrapper(Wire);

// 2) Software I²C on pins 2=SDA, 3=SCL
// SoftwareWire softBus(2, 3);
// SoftwareWireWrapper i2cWrapper(softBus);

SW35xx device(i2cWrapper);

void setup()
{
  Serial.begin(9600);

  device.begin();

  // device.setMaxCurrent5A();
  device.resetPDLimits();
}

void loop()
{
  Serial.println("Reading Status: ");
  device.readStatus();
  Serial.println("=======================================");
  Serial.print("Current input voltage: ");
  Serial.print(device.vin_mV);
  Serial.println(" mV");
  Serial.print("Current output voltage: ");
  Serial.print(device.vout_mV);
  Serial.println(" mV");
  Serial.print("Current USB-C current: ");
  Serial.print(device.iout_usbc_mA);
  Serial.println(" mA");
  Serial.print("Current USB-A current: ");
  Serial.print(device.iout_usba_mA);
  Serial.println(" mA");
  Serial.print("Current fast charge type: ");
  Serial.println(device.fastChargeTypeToString(device.fastChargeType));
  if (device.fastChargeType == SW35xx::PD_FIX || device.fastChargeType == SW35xx::PD_PPS)
  {
    Serial.print("Current PD version: ");
    Serial.println(device.PDVersion);
  }
  Serial.println("=======================================");
  Serial.println();
  delay(3000);
}
