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

#define PRINT_INTERVAL 4000UL
unsigned long lastPrint = 0;

void reportStatus()
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

  PowerStatus s = device.getPowerStatus();
  Serial.print(F("Buck converter : "));
  Serial.println(s.buckOn ? F("on") : F("off"));
  Serial.print(F("Port1 (USB-C): "));
  Serial.println(s.port1On ? F("on") : F("off"));
  Serial.print(F("Port2 (USB-A): "));
  Serial.println(s.port2On ? F("on") : F("off"));

  Serial.print("Power Limit: ");
  Serial.println(device.powerLimitToString(device.getPowerLimit()));

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
      Serial.println(F("\nChoose non‑PD power limit:"));
      Serial.println(F("0: 18W"));
      Serial.println(F("1: 24W"));
      Serial.println(F("2: 36W"));
      Serial.println(F("3: 60W"));
      Serial.print(F("> "));
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