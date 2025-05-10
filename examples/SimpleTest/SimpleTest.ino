#include <Arduino.h>
#include <SoftwareWire.h>
#include <SW35xx_lib.h>
#include "SoftwareWireWrapper.h"
using namespace SW35xx_lib;

// SW35xx sw(Wire);

// Create a SoftwareWire instance (choose your SDA and SCL pins)
SoftwareWire softWire(2, 3);
// Wrap the SoftwareWire instance
SoftwareWireWrapper softWrapper(softWire);
// Instantiate the SW35xx device with the wrapper
SW35xx device(softWrapper);

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
  Serial.begin(115200);
  softWrapper.begin(); // Initialize the SoftwareWire bus
  device.begin();
  Serial.println("SW35xx device initialized using SoftwareWire.");
  // Wire.begin(D1, D2);
  // sw.setMaxCurrent5A();
}

void loop()
{
  // sw.readStatus();
  device.readStatus();
  Serial.println("=======================================");
  Serial.printf("Current input voltage:%dmV\n", device.vin_mV);
  Serial.printf("Current output voltage:%dmV\n", device.vout_mV);
  Serial.printf("Current USB-C current:%dmA\r\n", device.iout_usbc_mA);
  Serial.printf("Current USB-A current:%dmA\r\n", device.iout_usba_mA);
  Serial.printf("Current fast charge type:%s\n", fastChargeType2String(device.fastChargeType));
  if (device.fastChargeType == SW35xx::PD_FIX || device.fastChargeType == SW35xx::PD_PPS)
    Serial.printf("Current PD version:%d\n", device.PDVersion);
  Serial.println("=======================================");
  Serial.println("");
  delay(2000);
}
