#include <Arduino.h>
#include "SW35xx_lib.h"
#include <Wire.h>

using namespace SW35xx_lib;

SW35xx sw(Wire);

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
  Wire.begin(); // A4=A SDA, A5=A SCL on Pro Mini
  // sw.setMaxCurrent5A();
  // sw.resetPDLimits();
}

void loop()
{
  Serial.println("Reading Status: ");
  sw.readStatus();
  Serial.println("=======================================");
  Serial.print("Current input voltage: ");
  Serial.print(sw.vin_mV);
  Serial.println(" mV");
  Serial.print("Current output voltage: ");
  Serial.print(sw.vout_mV);
  Serial.println(" mV");
  Serial.print("Current USB-C current: ");
  Serial.print(sw.iout_usbc_mA);
  Serial.println(" mA");
  Serial.print("Current USB-A current: ");
  Serial.print(sw.iout_usba_mA);
  Serial.println(" mA");
  Serial.print("Current fast charge type: ");
  Serial.println(fastChargeType2String(sw.fastChargeType));
  if (sw.fastChargeType == SW35xx::PD_FIX || sw.fastChargeType == SW35xx::PD_PPS)
  {
    Serial.print("Current PD version: ");
    Serial.println(sw.PDVersion);
  }
  Serial.println("=======================================");
  Serial.println();
  delay(3000);
}
