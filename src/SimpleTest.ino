#include <Arduino.h>
#include <SoftwareWire.h>
#include <SW35xx_lib.h>
#include "SerialUtils.h"
#include "TwoWireWrapper.h"
#include "SoftwareWireWrapper.h"

using namespace SW35xx_lib;

// 1) Hardware I²C
TwoWireWrapper i2cWrapper(Wire);

// 2) Software I²C on pins 2=SDA, 3=SCL
// SoftwareWire softBus(2, 3);
// SoftwareWireWrapper i2cWrapper(softBus);

SW35xx device(i2cWrapper);

#define PRINT_INTERVAL 5000UL
unsigned long lastPrint = 0;

void reportStatus()
{
  Serial.println("Reading Status: ");
  device.readStatus();
  Serial.println("=======================================");
  serial_printf(Serial, "Input voltage : %d mV\n", device.vin_mV);
  serial_printf(Serial, "Output voltage: %d mV\n", device.vout_mV);
  serial_printf(Serial, "USB-C current : %d mA\n", device.iout_usbc_mA);
  serial_printf(Serial, "USB-A current : %d mA\n", device.iout_usba_mA);
  Serial.println();

  uint8_t ver = device.getChipVersion();
  if (ver == 0xFF)
    Serial.println("Chip version: ERROR");
  else
    serial_printf(Serial, "Chip version    : %d\n", ver);

  SW35xx::FastChargeInfo fc = device.getFastChargeInfo();
  serial_printf(Serial, "Fast-charge LED : %s\n", boolToOnOff(fc.ledOn));
  serial_printf(Serial, "Protocol        : %s\n",
                device.fastChargeTypeToString(fc.protocol));
  if (fc.protocol == SW35xx::PD_FIX || fc.protocol == SW35xx::PD_PPS)
  {
    serial_printf(Serial, "PD Version      : %d\n", fc.pdVersion);
  }

  SW35xx::SwitchStatus s = device.getSwitchStatus();
  serial_printf(Serial, "Buck: %s, Port1-C: %s, Port2-A %s\n",
                boolToOnOff(s.buckOn), boolToOnOff(s.portAOn), boolToOnOff(s.portCOn));

  SW35xx::PresenceStatus ps = device.getPresenceStatus();
  serial_printf(Serial, "Port presence: %s\n", SW35xx::presenceStatusToString(ps));

  serial_printf(Serial, "Vin ADC enabled : %s\n", boolToOnOff(device.isVinAdcEnabled()));
  serial_printf(Serial, "Vin temp source : %s\n",
                device.getVinTempSource() == SW35xx::ADCVTS_NTC ? "NTC" : "45°C");

  serial_printf(Serial, "Power Limit     : %s\n", device.powerLimitToString(device.getPowerLimit()));

  serial_printf(Serial, "QC3.0 enabled   : %s\n", boolToOnOff(device.isQc3Enabled()));

  serial_printf(Serial, "Port config     : %s\n", SW35xx::portConfigToString(device.getPortConfig()));

  serial_printf(Serial, "Samsung 1.2 V  : %s\n", boolToOnOff(device.isSamsung12VModeEnabled()));

  serial_printf(Serial, "VID high byte  : %d\n", device.getVidHigh());

  Serial.println("=======================================");
}

void printMenu()
{
  Serial.println(F("\n=== MENU ==="));
  Serial.println(F("1: setMaxCurrent5A()"));
  Serial.println(F("2: resetPDLimits()"));
  Serial.println(F("3: setPowerLimit()"));
  Serial.println(F("4: Toggle Vin ADC enable"));
  Serial.println(F("5: Toggle Vin temp source (NTC/45C)"));
  Serial.println(F("6: Toggle QC3.0 enable"));
  Serial.println(F("7: setPortConfig()"));
  Serial.println(F("8: Toggle Samsung 1.2 V mode"));
  Serial.println(F("9: Set VID high byte"));
  Serial.println(F("x: exit menu"));
  Serial.print(F("> "));
}

void showMenu()
{
  printMenu();
  while (true)
  {
    if (!Serial.available())
    {
      delay(10);
      continue;
    }
    char c = Serial.read();
    if (c == '\n' || c == '\r')
      continue;
    switch (c)
    {
    case '1':
      device.setMaxCurrent5A();
      Serial.println(F("→ Set all PD currents to 5 A"));
      printMenu();
      break;
    case '2':
      device.resetPDLimits();
      Serial.println(F("→ PD limits reset to defaults"));
      printMenu();
      break;
    case '3':
    {
      // Prompt for new power limit
      serial_printf(Serial, "\nChoose non‑PD power limit:\n0: 18W\n1: 24W\n2: 36W\n3: 60W\n> ");
      // wait for a digit
      while (!Serial.available())
      {
        delay(10);
      }
      int choice = Serial.parseInt();
      // clamp to valid range
      if (choice < 0)
        choice = 0;
      if (choice > 3)
        choice = 3;
      // apply
      device.setPowerLimit((SW35xx::PowerLimit_t)choice);
      Serial.print(F("→ Power limit set to "));
      Serial.println(device.powerLimitToString(device.getPowerLimit()));
      printMenu();
      break;
    }
    case '4':
    {
      bool on = device.isVinAdcEnabled();
      device.enableVinAdc(!on);
      serial_printf(Serial, "→ Vin ADC %s\n", boolToOnOff(!on));
      printMenu();
      break;
    }
    case '5':
    {
      SW35xx::ADCVinTempSource_t cur = device.getVinTempSource();
      SW35xx::ADCVinTempSource_t nxt = (cur == SW35xx::ADCVTS_NTC)
                                           ? SW35xx::ADCVTS_45C
                                           : SW35xx::ADCVTS_NTC;
      device.setVinTempSource(nxt);
      serial_printf(Serial, "→ Vin temp source: %s\n",
                    nxt == SW35xx::ADCVTS_NTC ? "NTC" : "45°C");
      printMenu();
      break;
    }
    case '6':
    {
      bool on = device.isQc3Enabled();
      device.enableQc3(!on);
      serial_printf(Serial, "→ QC3.0 %s\n", boolToOnOff(!on));
      printMenu();
      break;
    }
    case '7':
    {
      Serial.println(F("\nSelect port config:"));
      Serial.println(F("0: Single A"));
      Serial.println(F("1: Dual A"));
      Serial.println(F("2: Single C"));
      Serial.println(F("3: AC (A+C)"));
      Serial.print(F("> "));
      while (!Serial.available())
        delay(10);
      int choice = Serial.parseInt();
      if (choice < 0)
        choice = 0;
      if (choice > 3)
        choice = 3;
      device.setPortConfig((SW35xx::PortConfig_t)choice);
      serial_printf(Serial, "→ Port config set to %s\n",
                    SW35xx::portConfigToString((SW35xx::PortConfig_t)choice));
      printMenu();
      break;
    }
    case '8':
    {
      bool on = device.isSamsung12VModeEnabled();
      device.enableSamsung12VMode(!on);
      serial_printf(Serial, "→ Samsung 1.2 V mode %s\n", boolToOnOff(!on));
      printMenu();
      break;
    }
    case '9':
    {
      serial_printf(Serial, "\nEnter VID high byte (0–255):\n> ");
      while (!Serial.available())
        delay(10);
      int val = Serial.parseInt();
      if (val < 0)
        val = 0;
      if (val > 255)
        val = 255;
      device.setVidHigh((uint8_t)val);
      serial_printf(Serial, "→ VID high set to %d\n", (uint8_t)val);
      printMenu();
      break;
    }
    case 'x':
    case 'X':
      Serial.println(F("Exiting menu.\n"));
      while (Serial.available())
        Serial.read();
      lastPrint = millis();
      return;
    default:
      Serial.println(F("Invalid choice"));
      printMenu();
      break;
    }
  }
}

void setup()
{
  Serial.begin(9600);
  while (!Serial)
  { /* wait */
  }
  device.begin();
  Serial.println(F("SW35xx interface ready."));
}

void loop()
{
  unsigned long now = millis();
  if (now - lastPrint >= PRINT_INTERVAL)
  {
    lastPrint = now;
    reportStatus();
  }
  if (Serial.available())
  {
    showMenu();
  }
  delay(20);
}