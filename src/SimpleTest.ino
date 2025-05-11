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
  serial_printf(Serial, "Current fast charge type: %s\n", device.fastChargeTypeToString(device.fastChargeType));

  if (device.fastChargeType == SW35xx::PD_FIX || device.fastChargeType == SW35xx::PD_PPS)
  {
    serial_printf(Serial, "Current PD version: %s\n", device.PDVersion);
  }

  uint8_t ver = device.getChipVersion();
  if (ver == 0xFF)
    Serial.println("Chip version: ERROR");
  else
    serial_printf(Serial, "Chip version : %d\n", ver);

  PowerStatus s = device.getPowerStatus();
  serial_printf(Serial, "Buck: %s, Port1-C: %s, Port2-A %s\n",
                boolToOnOff(s.buckOn), boolToOnOff(s.port1On), boolToOnOff(s.port2On));
  serial_printf(Serial, "Power Limit: %s\n", device.powerLimitToString(device.getPowerLimit()));

  Serial.println("=======================================");
}

void printMenu()
{
  Serial.println(F("\n=== MENU ==="));
  Serial.println(F("1: setMaxCurrent5A()"));
  Serial.println(F("2: resetPDLimits()"));
  Serial.println(F("3: setPowerLimit()"));
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