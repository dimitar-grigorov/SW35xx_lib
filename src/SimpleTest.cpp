#include <Arduino.h>
#include <SoftwareWire.h>
#include <SW35xx_lib.h>
#include "SerialUtils.h"
#include "TwoWireWrapper.h"
#include "SoftwareWireWrapper.h"
#include <avr/pgmspace.h>

using namespace SW35xx_lib;

// 1) Hardware I²C
TwoWireWrapper i2cWrapper(Wire);

// 2) Software I²C on pins 2=SDA, 3=SCL
// SoftwareWire softBus(2, 3);
// SoftwareWireWrapper i2cWrapper(softBus);

SW35xx device(i2cWrapper);

#define PRINT_INTERVAL 5000UL
unsigned long lastPrint = 0;

// Menu items
static const char str1[] PROGMEM = "setMaxCurrent5A()";
static const char str2[] PROGMEM = "resetPDLimits()";
static const char str3[] PROGMEM = "setPowerLimit()";
static const char str4[] PROGMEM = "Toggle Vin ADC enable";
static const char str5[] PROGMEM = "Toggle Vin temp src";
static const char str6[] PROGMEM = "Toggle QC3.0 enable";
static const char str7[] PROGMEM = "setPortConfig()";
static const char str8[] PROGMEM = "Toggle Samsung 1.2V";
static const char str9[] PROGMEM = "Set VID high byte";
static const char strA[] PROGMEM = "Toggle DPDM support";
static const char strB[] PROGMEM = "Set dual-port limit";
static const char strD[] PROGMEM = "Set VID low byte";

struct MenuItem
{
  char key;
  PGM_P desc;
  void (*action)();
};

// Forward declaration of action functions
static void actionSetMax5A();
static void actionResetPD();
static void actionSetPowerLimit();
static void actionToggleVinAdc();
static void actionToggleVinTemp();
static void actionToggleQc3();
static void actionSetPortConfig();
static void actionToggleSamsung12V();
static void actionSetVidHigh();
static void actionToggleDpdm();
static void actionSetDualPortLimit();
static void actionSetVidLow();

// Table with plain pointer constants
static const MenuItem MENU[] = {
    {'1', str1, actionSetMax5A},
    {'2', str2, actionResetPD},
    {'3', str3, actionSetPowerLimit},
    {'4', str4, actionToggleVinAdc},
    {'5', str5, actionToggleVinTemp},
    {'6', str6, actionToggleQc3},
    {'7', str7, actionSetPortConfig},
    {'8', str8, actionToggleSamsung12V},
    {'9', str9, actionSetVidHigh},
    {'A', strA, actionToggleDpdm},
    {'B', strB, actionSetDualPortLimit},
    {'D', strD, actionSetVidLow}};

void printMenu()
{
  Serial.println(F("\n=== MENU ==="));
  for (uint8_t i = 0; i < sizeof(MENU) / sizeof(MENU[0]); i++)
  {
    Serial.print(MENU[i].key);
    Serial.print(F(": "));
    Serial.println(
        reinterpret_cast<const __FlashStringHelper *>(MENU[i].desc));
  }
  Serial.println(F("X: Exit menu"));
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
    if (c == 'X' || c == 'x')
    {
      Serial.println(F("Exit menu."));
      lastPrint = millis();
      break;
    }
    // Dispatch:
    for (auto &mi : MENU)
    {
      if (c == mi.key)
      {
        mi.action();
        break;
      }
    }
    printMenu();
  }
}

// --- ACTION FUNCTIONS ---

static void actionSetMax5A()
{
  device.setMaxCurrent5A();
  Serial.println(F("→ Set all PD currents to 5 A"));
}

static void actionResetPD()
{
  device.resetPDLimits();
  Serial.println(F("→ PD limits reset to defaults"));
}

static void actionSetPowerLimit()
{
  Serial.println(F("\nChoose non‑PD power limit:"));
  Serial.println(F("0: 18W"));
  Serial.println(F("1: 24W"));
  Serial.println(F("2: 36W"));
  Serial.println(F("3: 60W"));
  Serial.print(F("> "));
  while (!Serial.available())
    delay(10);
  int choice = Serial.parseInt();
  choice = constrain(choice, 0, 3);
  device.setPowerLimit((SW35xx::PowerLimit_t)choice);
  Serial.print(F("→ Power limit set to "));
  Serial.println(device.powerLimitToString(device.getPowerLimit()));
}

static void actionToggleVinAdc()
{
  bool on = device.isVinAdcEnabled();
  device.enableVinAdc(!on);
  Serial.print(F("→ Vin ADC "));
  Serial.println(on ? F("Off") : F("On"));
}

static void actionToggleVinTemp()
{
  auto cur = device.getVinTempSource();
  auto nxt = (cur == SW35xx::ADCVTS_NTC) ? SW35xx::ADCVTS_45C : SW35xx::ADCVTS_NTC;
  device.setVinTempSource(nxt);
  Serial.print(F("→ Vin temp source: "));
  Serial.println(nxt == SW35xx::ADCVTS_NTC ? F("NTC") : F("45°C"));
}

static void actionToggleQc3()
{
  bool on = device.isQc3Enabled();
  device.enableQc3(!on);
  Serial.print(F("→ QC3.0 "));
  Serial.println(on ? F("Off") : F("On"));
}

static void actionSetPortConfig()
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
  choice = constrain(choice, 0, 3);
  device.setPortConfig((SW35xx::PortConfig_t)choice);
  Serial.print(F("→ Port config set to "));
  Serial.println(SW35xx::portConfigToString((SW35xx::PortConfig_t)choice));
}

static void actionToggleSamsung12V()
{
  bool on = device.isSamsung12VModeEnabled();
  device.enableSamsung12VMode(!on);
  Serial.print(F("→ Samsung 1.2 V mode "));
  Serial.println(on ? F("Off") : F("On"));
}

static void actionSetVidHigh()
{
  Serial.print(F("\nEnter VID high byte (0–255):\n> "));
  while (!Serial.available())
    delay(10);
  int val = Serial.parseInt();
  val = constrain(val, 0, 255);
  device.setVidHigh((uint8_t)val);
  Serial.print(F("→ VID high set to "));
  Serial.println(val);
}

static void actionToggleDpdm()
{
  bool on = device.isDpdmEnabled();
  device.enableDpdm(!on);
  Serial.print(F("→ DPDM support "));
  Serial.println(on ? F("Off") : F("On"));
}

static void actionSetDualPortLimit()
{
  Serial.println(F("\nSelect dual-port limit:"));
  Serial.println(F("0: 2.6A each"));
  Serial.println(F("1: 2.2A each"));
  Serial.println(F("2: 1.7A each"));
  Serial.println(F("3: 3.2A each"));
  Serial.print(F("> "));
  while (!Serial.available())
    delay(10);
  int choice = Serial.parseInt();
  choice = constrain(choice, 0, 3);
  device.setDualPortLimit((SW35xx::DualPortLimit_t)choice);
  Serial.print(F("→ Dual-port limit set to "));
  Serial.println(SW35xx::dualPortLimitToString((SW35xx::DualPortLimit_t)choice));
}

static void actionSetVidLow()
{
  Serial.print(F("\nEnter VID low byte (hex 00–FF):\n> "));
  while (!Serial.available())
    delay(10);
  String s = Serial.readStringUntil('\n');
  uint8_t v = (uint8_t)strtoul(s.c_str(), nullptr, 16);
  device.setVidLow(v);
  Serial.print(F("→ VID low set to 0x"));
  if (v < 0x10)
    Serial.print('0');
  Serial.println(v, HEX);
}

// --- END OF ACTION FUNCTIONS ---

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
  serial_printf(Serial, "Samsung 1.2 V   : %s\n", boolToOnOff(device.isSamsung12VModeEnabled()));
  serial_printf(Serial, "VID high byte   : %d\n", device.getVidHigh());
  serial_printf(Serial, "DPDM support    : %s\n", boolToOnOff(device.isDpdmEnabled()));
  serial_printf(Serial, "Dual-port limit : %s\n", device.dualPortLimitToString(device.getDualPortLimit()));
  SW35xx::QCConfig1 qc1 = device.getQuickChargeConfig1();
  serial_printf(Serial, "QC-cfg1 flags   : %s\n", SW35xx::quickChargeConfig1ToString(qc1));
  serial_printf(Serial, "VID low byte    : %d\n", device.getVidLow());

  Serial.println("=======================================");
}

void setup()
{
  Serial.begin(9600);
  while (!Serial)
  {
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