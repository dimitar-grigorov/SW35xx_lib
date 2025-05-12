#include <Arduino.h>
#include "SW35xx_lib.h"

#define SW35XX_ADDRESS 0x3c
#define SW35XX_IC_VERSION 0x01
#define SW35XX_FCX_STATUS 0x06
#define SW35XX_SYS_STATUS0 0x07 // System Status 0: buck/C/A on-off
#define SW35XX_SYS_STATUS1 0x08 // System Status 1: device-present state
#define SW35XX_I2C_ENABLE 0x12
#define SW35XX_ADC_CTRL 0x13
#define SW35XX_ADC_VIN_H 0x30            // Not re-implemented
#define SW35XX_ADC_VOUT_H 0x31           // Not re-implemented
#define SW35XX_ADC_VIN_VOUT_L 0x32       // Not re-implemented
#define SW35XX_ADC_IOUT_USBC_H 0x33      // Not re-implemented
#define SW35XX_ADC_IOUT_USBA_H 0x34      // Not re-implemented
#define SW35XX_ADC_IOUT_USBC_USBA_L 0x35 // Not re-implemented
#define SW35XX_ADC_TS_H 0x37             // Not re-implemented
#define SW35XX_ADC_TS_L 0x38             // Not re-implemented
#define SW35XX_ADC_DATA_TYPE 0x3a        // Not re-implemented
#define SW35XX_ADC_DATA_BUF_H 0x3b       // Not re-implemented
#define SW35XX_ADC_DATA_BUF_L 0x3c       // Not re-implemented
#define SW35XX_PD_SRC_REQ 0x70           // Not re-implemented
#define SW35XX_PD_CMD_EN 0x71            // Not implemented
#define SW35XX_PWR_CONF 0xa6             // Implemented
#define SW35XX_QC_CONF0 0xaa             // Implemented
#define SW35XX_PORT_CONFIG 0xab          // Implemented
#define SW35XX_QC_CFG1 0xad              // Implemented
#define SW35XX_VID_CONF0 0xaf            // Implemented
#define SW35XX_PD_CONF1 0xb0             // Not re-implemented
#define SW35XX_PD_CONF2 0xb1             // Not re-implemented
#define SW35XX_PD_CONF3 0xb2             // Not re-implemented
#define SW35XX_PD_CONF4 0xb3             // Not re-implemented
#define SW35XX_PD_CONF5 0xb4             // Not re-implemented
#define SW35XX_PD_CONF6 0xb5             // Not re-implemented
#define SW35XX_PD_CONF7 0xb6             // Not re-implemented
#define SW35XX_PD_CONF8 0xb7             // Not re-implemented
#define SW35XX_PD_CONF9 0xb8             // Not re-implemented
#define SW35XX_QC_CONF1 0xb9             // Implemented - needs human friendly demo
#define SW35XX_QC_CONF2 0xba             // Not re-implemented
#define SW35XX_QC_CONF3 0xbc             // Not implemented
#define SW35XX_PD_CONF10 0xbe            // Not re-implemented
#define SW35XX_CUR_LIMIT_CFG 0xbd        // Implemented
#define SW35XX_VID_CONF1 0xbf

#define I2C_RETRIES 10

namespace SW35xx_lib
{

  SW35xx::SW35xx(I2CInterface &i2c) : _i2c(i2c) {}
  // SW35xx::SW35xx(TwoWire &i2c) : _i2c(i2c) {}
  SW35xx::~SW35xx() {}

  int SW35xx::i2cReadReg8(const uint8_t reg)
  {
    for (int i = 0; i < I2C_RETRIES; i++)
    {
      _i2c.beginTransmission(SW35XX_ADDRESS);
      if (_i2c.write(reg) != 1)
      {
        continue;
      }
      if (_i2c.endTransmission() != 0)
      {
        continue;
      }

      if (_i2c.requestFrom(SW35XX_ADDRESS, 1) != 1)
      {
        continue;
      }

      // Wait until data is available if required
      for (int k = 0; !_i2c.available() && k < I2C_RETRIES; k++)
      {
        delay(10);
      }

      const int value = _i2c.read();
      if (value < 0)
      {
        continue;
      }
      return value;
    }

    return 0;
  }

  int SW35xx::i2cWriteReg8(const uint8_t reg, const uint8_t data)
  {
    int error = -1;
    for (int i = 0; i < I2C_RETRIES; i++)
    {
      _i2c.beginTransmission(SW35XX_ADDRESS);
      if (_i2c.write(reg) != 1)
      {
        continue;
      }
      if (_i2c.write(data) != 1)
      {
        continue;
      }
      error = _i2c.endTransmission();
      if (error == 0)
      {
        return 0;
      }
    }
    return error;
  }

  void SW35xx::begin()
  {
    _i2c.begin();
    // Enable input voltage reading

    // i2cWriteReg8(SW35XX_ADC_CTRL, 0x02);
    enableVinAdc(true);
  }

  uint16_t SW35xx::readADCDataBuffer(const enum ADCDataType type)
  {
    i2cWriteReg8(SW35XX_ADC_DATA_TYPE, type);
    uint16_t value = i2cReadReg8(SW35XX_ADC_DATA_BUF_H) << 4;
    value |= i2cReadReg8(SW35XX_ADC_DATA_BUF_L) | 0x0f;
    return value;
  }

  uint8_t SW35xx::getChipVersion()
  {
    // read REG 0x01
    int tmp = i2cReadReg8(SW35XX_IC_VERSION);
    if (tmp < 0)
    {
      // error
      return 0xFF;
    }
    // bits [2:0] = version
    return (uint8_t)(tmp & 0x07);
  }

  // Read out and decode REG 0x06: Fast charge indicator status
  SW35xx::FastChargeInfo SW35xx::getFastChargeInfo()
  {
    FastChargeInfo info = {false, 0, NOT_FAST_CHARGE};

    // 1) Read the raw status byte
    int tmpStatus = i2cReadReg8(SW35XX_FCX_STATUS);
    if (tmpStatus < 0)
    {
      // I²C error, leave everything zero/false
      return info;
    }
    uint8_t status = (uint8_t)tmpStatus;

    // 2) Bit 7: fast-charge LED state (0 = off, 1 = on)
    info.ledOn = (status & 0x80) != 0;

    // bits 5–4 = PD version: 1 → PD2.0, 2 → PD3.0, else = none
    uint8_t pdVer = (status >> 4) & 0x03;
    switch (pdVer)
    {
    case 1:
      info.pdVersion = 2;
      break;
    case 2:
      info.pdVersion = 3;
      break;
    default:
      info.pdVersion = 0;
      break;
    }

    // 4) Bits 3–0: fast-charge protocol indicator
    //    1 = QC2.0, 2 = QC3.0, 3 = FCP, 4 = SCP,
    //    5 = PD FIX, 6 = PD PPS, 7 = PE1.1, 8 = PE2.0,
    //    9 = VOOC, 10 = SFCP, 11 = AFC, else = reserved
    info.protocol = (fastChargeType_t)(status & 0x0F);

    return info;
  }

  SW35xx::SwitchStatus SW35xx::getSwitchStatus()
  {
    int tmp = i2cReadReg8(SW35XX_SYS_STATUS0);
    if (tmp < 0)
    {
      // on I²C error, return all false
      return SwitchStatus{false, false, false};
    }
    uint8_t r = (uint8_t)tmp;

    return SwitchStatus{
        /* buckOn   */ (r & BIT(2)) != 0,
        /* portCOn  */ (r & BIT(0)) != 0,
        /* portAOn  */ (r & BIT(1)) != 0};
  }

  SW35xx::PresenceStatus SW35xx::getPresenceStatus()
  {
    int raw = i2cReadReg8(SW35XX_SYS_STATUS1);
    if (raw < 0)
    {
      return PresenceStatus::Unknown;
    }
    // take bits 7–4
    uint8_t code = ((uint8_t)raw >> 4) & 0x0F;
    // valid codes are 1..8
    if (code >= 1 && code <= 8)
    {
      return (PresenceStatus)code;
    }
    else
    {
      return PresenceStatus::Unknown;
    }
  }

  void SW35xx::enableI2CWrite()
  {
    i2cWriteReg8(SW35XX_I2C_ENABLE, 0x20);
    i2cWriteReg8(SW35XX_I2C_ENABLE, 0x40);
    i2cWriteReg8(SW35XX_I2C_ENABLE, 0x80);
  }

  void SW35xx::disableI2CWrite()
  {
    i2cWriteReg8(SW35XX_I2C_ENABLE, 0x00);
  }

  // Read bit 1 = Vin ADC enable
  bool SW35xx::isVinAdcEnabled()
  {
    int v = i2cReadReg8(SW35XX_ADC_CTRL);
    if (v < 0)
      return false;
    return ((uint8_t)v & BIT(1)) != 0;
  }

  // Write bit 1 = Vin ADC enable
  void SW35xx::enableVinAdc(bool enable)
  {
    // read-modify-write
    uint8_t cur = (uint8_t)i2cReadReg8(SW35XX_ADC_CTRL);
    if (enable)
      cur |= BIT(1);
    else
      cur &= ~BIT(1);
    i2cWriteReg8(SW35XX_ADC_CTRL, cur);
  }

  // Read bit 6 = temp source (0=NTC, 1=45°C)
  SW35xx::ADCVinTempSource_t SW35xx::getVinTempSource()
  {
    int v = i2cReadReg8(SW35XX_ADC_CTRL);
    if (v < 0)
      return ADCVTS_NTC;
    return (((uint8_t)v & BIT(6)) != 0)
               ? ADCVTS_45C
               : ADCVTS_NTC;
  }

  // Write bit 6 = temp source
  void SW35xx::setVinTempSource(ADCVinTempSource_t src)
  {
    uint8_t cur = (uint8_t)i2cReadReg8(SW35XX_ADC_CTRL);
    if (src == ADCVTS_45C)
      cur |= BIT(6);
    else
      cur &= ~BIT(6);
    i2cWriteReg8(SW35XX_ADC_CTRL, cur);
  }

  // — Read bits [1:0] of PWR_CONF
  SW35xx::PowerLimit_t SW35xx::getPowerLimit()
  {
    uint8_t r = i2cReadReg8(SW35XX_PWR_CONF);
    return (PowerLimit_t)(r & 0x03);
  }

  // — Write bits [1:0] of PWR_CONF, preserving the other bits —
  void SW35xx::setPowerLimit(PowerLimit_t lim)
  {
    uint8_t old = i2cReadReg8(SW35XX_PWR_CONF);
    // pull only the low two bits
    uint8_t nw = (old & 0xFC) | ((uint8_t)lim & 0x03);

    enableI2CWrite();
    i2cWriteReg8(SW35XX_PWR_CONF, nw);
    disableI2CWrite();
  }

  bool SW35xx::isQc3Enabled()
  {
    int v = i2cReadReg8(SW35XX_QC_CONF0);
    if (v < 0)
      return false;
    return ((uint8_t)v & BIT(6)) != 0;
  }

  void SW35xx::enableQc3(bool enable)
  {
    // read-modify-write
    uint8_t cur = (uint8_t)i2cReadReg8(SW35XX_QC_CONF0);
    if (enable)
      cur |= BIT(6);
    else
      cur &= ~BIT(6);
    enableI2CWrite(); // unlock extended writes
    i2cWriteReg8(SW35XX_QC_CONF0, cur);
    disableI2CWrite(); // re-lock
  }

  SW35xx::PortConfig_t SW35xx::getPortConfig()
  {
    int v = i2cReadReg8(SW35XX_PORT_CONFIG);
    if (v < 0)
      return PORT_SINGLE_A;
    uint8_t raw = ((uint8_t)v >> 2) & 0x03;
    return (PortConfig_t)raw;
  }

  void SW35xx::setPortConfig(PortConfig_t cfg)
  {
    // read-modify-write, preserving reserved bits
    uint8_t cur = (uint8_t)i2cReadReg8(SW35XX_PORT_CONFIG);
    cur = (cur & ~BIT(2) & ~BIT(3)) | ((uint8_t)cfg << 2);
    enableI2CWrite();
    i2cWriteReg8(SW35XX_PORT_CONFIG, cur);
    disableI2CWrite();
  }

  // — REG 0xAD: Samsung 1.2 V mode —
  bool SW35xx::isSamsung12VModeEnabled()
  {
    int v = i2cReadReg8(SW35XX_QC_CFG1);
    if (v < 0)
      return false;
    return ((uint8_t)v & BIT(2)) != 0;
  }

  void SW35xx::enableSamsung12VMode(bool enable)
  {
    uint8_t cur = (uint8_t)i2cReadReg8(SW35XX_QC_CFG1);
    if (enable)
      cur |= BIT(2);
    else
      cur &= ~BIT(2);
    enableI2CWrite();
    i2cWriteReg8(SW35XX_QC_CFG1, cur);
    disableI2CWrite();
  }

  // — REG 0xAF: Vendor-ID high byte —
  uint8_t SW35xx::getVidHigh()
  {
    int v = i2cReadReg8(SW35XX_VID_CONF0);
    return (v < 0) ? 0 : (uint8_t)v;
  }

  void SW35xx::setVidHigh(uint8_t vidHigh)
  {
    enableI2CWrite();
    i2cWriteReg8(SW35XX_VID_CONF0, vidHigh);
    disableI2CWrite();
  }

  SW35xx::QCConfig1 SW35xx::getQuickChargeConfig1()
  {
    uint8_t b = (uint8_t)i2cReadReg8(SW35XX_QC_CONF1);
    return QCConfig1{
        /*cPortFastCharge=*/(b & BIT(7)) != 0,
        /*aPortFastCharge=*/(b & BIT(6)) != 0,
        /*pdProtocol     =*/(b & BIT(5)) != 0,
        /*qcProtocol     =*/(b & BIT(4)) != 0,
        /*fcpProtocol    =*/(b & BIT(3)) != 0,
        /*scpProtocol    =*/(b & BIT(2)) != 0,
        /*peProtocol     =*/(b & BIT(0)) != 0};
  }

  void SW35xx::setQuickChargeConfig1(const QCConfig1 &cfg)
  {
    uint8_t b = 0;
    b |= (uint8_t)cfg.cPortFastCharge << 7;
    b |= (uint8_t)cfg.aPortFastCharge << 6;
    b |= (uint8_t)cfg.pdProtocol << 5;
    b |= (uint8_t)cfg.qcProtocol << 4;
    b |= (uint8_t)cfg.fcpProtocol << 3;
    b |= (uint8_t)cfg.scpProtocol << 2;
    b |= (uint8_t)cfg.peProtocol << 0;

    enableI2CWrite();
    i2cWriteReg8(SW35XX_QC_CONF1, b);
    disableI2CWrite();
  }

  // pull the format into PROGMEM
  static const char qc_fmt[] PROGMEM =
      "C-port:%s;A-port:%s;PD:%s;QC:%s;FCP:%s;SCP:%s;PE:%s";

  // now define the static member
  const char *SW35xx::quickChargeConfig1ToString(const QCConfig1 &cfg)
  {
    static char buf[40];
    const char *on = "On";
    const char *off = "Off";
    snprintf_P(buf, sizeof(buf), qc_fmt,
               cfg.cPortFastCharge ? on : off,
               cfg.aPortFastCharge ? on : off,
               cfg.pdProtocol ? on : off,
               cfg.qcProtocol ? on : off,
               cfg.fcpProtocol ? on : off,
               cfg.scpProtocol ? on : off,
               cfg.peProtocol ? on : off);
    return buf;
  }

  bool SW35xx::isDpdmEnabled()
  {
    int v = i2cReadReg8(SW35XX_CUR_LIMIT_CFG);
    if (v < 0)
      return false;
    return ((uint8_t)v & BIT(6)) != 0;
  }

  void SW35xx::enableDpdm(bool enable)
  {
    uint8_t cur = (uint8_t)i2cReadReg8(SW35XX_CUR_LIMIT_CFG);
    if (enable)
      cur |= BIT(6);
    else
      cur &= ~BIT(6);
    enableI2CWrite();
    i2cWriteReg8(SW35XX_CUR_LIMIT_CFG, cur);
    disableI2CWrite();
  }

  SW35xx::DualPortLimit_t SW35xx::getDualPortLimit()
  {
    int v = i2cReadReg8(SW35XX_CUR_LIMIT_CFG);
    if (v < 0)
      return DPL_2_6A;
    uint8_t sel = ((uint8_t)v >> 4) & 0x03;
    return (DualPortLimit_t)sel;
  }

  void SW35xx::setDualPortLimit(DualPortLimit_t lim)
  {
    uint8_t cur = (uint8_t)i2cReadReg8(SW35XX_CUR_LIMIT_CFG);
    // clear bits 5–4, then set
    cur = (cur & ~(BIT(5) | BIT(4))) | ((uint8_t)lim << 4);
    enableI2CWrite();
    i2cWriteReg8(SW35XX_CUR_LIMIT_CFG, cur);
    disableI2CWrite();
  }

  uint8_t SW35xx::getVidLow()
  {
    int v = i2cReadReg8(SW35XX_VID_CONF1);
    return (v < 0) ? 0 : (uint8_t)v;
  }

  void SW35xx::setVidLow(uint8_t vidLow)
  {
    enableI2CWrite();
    i2cWriteReg8(SW35XX_VID_CONF1, vidLow);
    disableI2CWrite();
  }

  void SW35xx::readStatus(const bool useADCDataBuffer)
  {
    uint16_t vin = 0;
    uint16_t vout = 0;
    uint16_t iout_usbc = 0;
    uint16_t iout_usba = 0;

    if (useADCDataBuffer)
    {
      // Read input voltage
      vin = readADCDataBuffer(ADC_VIN);
      // Read output voltage
      vout = readADCDataBuffer(ADC_VOUT);
      // Read Port1 (USB-C) output current
      iout_usbc = readADCDataBuffer(ADC_IOUT_USB_C);
      // Read Port2 (USB-A) output current
      iout_usba = readADCDataBuffer(ADC_IOUT_USB_A);
    }
    else
    {
      const uint8_t vin_vout_low = i2cReadReg8(SW35XX_ADC_VIN_VOUT_L);
      vin = i2cReadReg8(SW35XX_ADC_VIN_H) << 4;
      vin |= vin_vout_low >> 4;
      vout = i2cReadReg8(SW35XX_ADC_VOUT_H) << 4;
      vout |= vin_vout_low & 0x0F;

      const uint8_t iout_low = i2cReadReg8(SW35XX_ADC_IOUT_USBC_USBA_L);
      iout_usbc = i2cReadReg8(SW35XX_ADC_IOUT_USBC_H) << 4;
      iout_usbc |= iout_low >> 4;
      iout_usba = i2cReadReg8(SW35XX_ADC_IOUT_USBA_H) << 4;
      iout_usba |= iout_low & 0x0F;
    }

    vin_mV = vin * 10;
    vout_mV = vout * 6;
    if (iout_usbc > 15) // The readed data when there is no output is 15
      iout_usbc_mA = iout_usbc * 5 / 2;
    else
      iout_usbc_mA = 0;

    if (iout_usba > 15)
      iout_usba_mA = iout_usba * 5 / 2;
    else
      iout_usba_mA = 0;
  }

  float SW35xx::readTemperature(const bool useADCDataBuffer)
  {
    uint16_t temperature = 0;

    if (useADCDataBuffer)
    {
      temperature = readADCDataBuffer(ADC_TEMPERATURE);
    }
    else
    {
      temperature = i2cReadReg8(SW35XX_ADC_TS_H) << 4;
      temperature |= i2cReadReg8(SW35XX_ADC_TS_L) & 0x0F;
    }

    /* return it in mV */
    return temperature * 0.5;
  }

  void SW35xx::sendPDCmd(SW35xx::PDCmd_t cmd)
  {
    i2cWriteReg8(SW35XX_PD_SRC_REQ, (const uint8_t)cmd);
    i2cWriteReg8(SW35XX_PD_SRC_REQ, (const uint8_t)(cmd | 0x80));
  }

  void SW35xx::rebroadcastPDO()
  {
    // TODO: Check if this works
    i2cWriteReg8(SW35XX_ADC_CTRL, 0x03);
  }

  void SW35xx::setMaxCurrent5A()
  {
    enableI2CWrite();
    i2cWriteReg8(SW35XX_PD_CONF1, 0b01100100);
    i2cWriteReg8(SW35XX_PD_CONF2, 0b01100100);
    i2cWriteReg8(SW35XX_PD_CONF3, 0b01100100);
    i2cWriteReg8(SW35XX_PD_CONF4, 0b01100100);
    i2cWriteReg8(SW35XX_PD_CONF6, 0b01100100);
    i2cWriteReg8(SW35XX_PD_CONF7, 0b01100100);
    disableI2CWrite();
  }

  void SW35xx::setQuickChargeConfiguration(const uint16_t flags,
                                           const enum QuickChargePowerClass power)
  {
    /* mask all available bits to avoid setting reserved bits */
    const uint16_t validFlags = flags & QC_CONF_ALL;
    const uint16_t validPower = power & QC_PWR_20V_2;
    const uint8_t conf1 = validFlags;
    const uint8_t conf2 = (validFlags >> 8) | (validPower << 2);

    enableI2CWrite();
    i2cWriteReg8(SW35XX_QC_CONF1, conf1);
    i2cWriteReg8(SW35XX_QC_CONF2, conf2);
    disableI2CWrite();
  }

  void SW35xx::setMaxCurrentsFixed(uint32_t ma_5v, uint32_t ma_9v, uint32_t ma_12v, uint32_t ma_15v, uint32_t ma_20v)
  {
    if (ma_5v > 5000)
      ma_5v = 5000;
    if (ma_9v > 5000)
      ma_9v = 5000;
    if (ma_12v > 5000)
      ma_12v = 5000;
    if (ma_15v > 5000)
      ma_15v = 5000;
    if (ma_20v > 5000)
      ma_20v = 5000;

    uint8_t tmp = i2cReadReg8(SW35XX_PD_CONF8);

    if (ma_9v == 0)
      tmp &= 0b11111011;
    else
      tmp |= 0b00000100;

    if (ma_12v == 0)
      tmp &= 0b11110111;
    else
      tmp |= 0b00001000;

    if (ma_15v == 0)
      tmp &= 0b11101111;
    else
      tmp |= 0b00010000;

    if (ma_20v == 0)
      tmp &= 0b11011111;
    else
      tmp |= 0b00100000;

    enableI2CWrite();

    i2cWriteReg8(SW35XX_PD_CONF8, tmp);
    i2cWriteReg8(SW35XX_PD_CONF1, ma_5v / 50);
    i2cWriteReg8(SW35XX_PD_CONF2, ma_9v / 50);
    i2cWriteReg8(SW35XX_PD_CONF3, ma_12v / 50);
    i2cWriteReg8(SW35XX_PD_CONF4, ma_15v / 50);
    i2cWriteReg8(SW35XX_PD_CONF5, ma_20v / 50);

    disableI2CWrite();
  }

  void SW35xx::setMaxCurrentsPPS(uint32_t ma_pps1, uint32_t ma_pps2)
  {
    if (ma_pps1 > 5000)
      ma_pps1 = 5000;
    if (ma_pps2 > 5000)
      ma_pps2 = 5000;
    uint8_t tmp = i2cReadReg8(SW35XX_PD_CONF8);

    if (ma_pps1 == 0)
      tmp &= 0b10111111;
    else
      tmp |= 0b01000000;

    if (ma_pps1 == 0)
      tmp &= 0b01111111;
    else
      tmp |= 0b10000000;

    enableI2CWrite();

    i2cWriteReg8(SW35XX_PD_CONF8, tmp);
    i2cWriteReg8(SW35XX_PD_CONF6, ma_pps1 / 50);
    i2cWriteReg8(SW35XX_PD_CONF7, ma_pps2 / 50);
    disableI2CWrite();
  }

  void SW35xx::resetPDLimits()
  {
    // 1) Unlock writes to PD_CONF registers
    enableI2CWrite();

    // 2) Restore each PD_CONF register to its default 0xFF
    const uint8_t pdRegs[] = {
        SW35XX_PD_CONF1, SW35XX_PD_CONF2, SW35XX_PD_CONF3,
        SW35XX_PD_CONF4, SW35XX_PD_CONF5, SW35XX_PD_CONF6,
        SW35XX_PD_CONF7, SW35XX_PD_CONF8, SW35XX_PD_CONF9,
        SW35XX_PD_CONF10};
    for (uint8_t reg : pdRegs)
    {
      i2cWriteReg8(reg, 0xFF);
    }

    // 3) Lock I²C-write again
    disableI2CWrite();

    // 4a) Re-broadcast the (now default) PDOs
    // rebroadcastPDO();

    // —or— 4b) force a PD hard-reset to renegotiate from scratch
    sendPDCmd(PDCmd_t::HARDRESET);
  }
} // namespace SW35xx_lib
