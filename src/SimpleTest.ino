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

const char *fastChargeType2String(SW35xx::fastChargeType_t fastChargeType)
{
  switch (fastChargeType)
  {
  case SW35xx::NOT_FAST_CHARGE:
    return "Not fast charge";
    break;
  case SW35xx::QC2:
    return "QC2.0";
    break;
  case SW35xx::QC3:
    return "QC3.0";
    break;
  case SW35xx::FCP:
    return "FCP";
    break;
  case SW35xx::SCP:
    return "SCP";
    break;
  case SW35xx::PD_FIX:
    return "PD Fix";
    break;
  case SW35xx::PD_PPS:
    return "PD PPS";
    break;
  case SW35xx::MTKPE1:
    return "MTK PE1.1";
    break;
  case SW35xx::MTKPE2:
    return "MTK PE2.0";
    break;
  case SW35xx::LVDC:
    return "LVDC";
    break;
  case SW35xx::SFCP:
    return "SFCP";
    break;
  case SW35xx::AFC:
    return "AFC";
    break;
  default:
    return "unknown";
    break;
  }
}

void setup()
{
  Serial.begin(9600);

  device.begin();

  // sw.setMaxCurrent5A();
  // sw.resetPDLimits();
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
  Serial.println(fastChargeType2String(device.fastChargeType));
  if (device.fastChargeType == SW35xx::PD_FIX || device.fastChargeType == SW35xx::PD_PPS)
  {
    Serial.print("Current PD version: ");
    Serial.println(device.PDVersion);
  }
  Serial.println("=======================================");
  Serial.println();
  delay(3000);
}
