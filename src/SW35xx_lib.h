#pragma once
#include <stdint.h>

#include "I2CInterface.h"
// #include <Wire.h>

#ifndef BIT
#define BIT(x) (1 << (x))
#endif

namespace SW35xx_lib
{

  struct PowerStatus
  {
    bool buckOn;
    bool port1On;
    bool port2On;
  };

  class SW35xx
  {
  public:
    enum fastChargeType_t
    {
      NOT_FAST_CHARGE = 0,
      QC2,
      QC3,
      FCP,
      SCP,
      PD_FIX,
      PD_PPS,
      MTKPE1,
      MTKPE2,
      LVDC,
      SFCP,
      AFC,
      FAST_CHARGE_TYPE_COUNT
    };

    inline const char *fastChargeTypeToString(fastChargeType_t aType)
    {
      // a compile-time array of names
      static const char *names[FAST_CHARGE_TYPE_COUNT] = {
          "Not fast charge",
          "QC2.0",
          "QC3.0",
          "FCP",
          "SCP",
          "PD Fix",
          "PD PPS",
          "MTK PE1.1",
          "MTK PE2.0",
          "LVDC",
          "SFCP",
          "AFC"};
      return (aType < FAST_CHARGE_TYPE_COUNT)
                 ? names[aType]
                 : "Unknown";
    }

    struct FastChargeInfo
    {
      bool ledOn;                ///< true if the fast-charge LED is lit (bit 7)
      uint8_t pdVersion;         ///< 2 for PD2.0, 3 for PD3.0 (bits 5–4 + 1)
      fastChargeType_t protocol; ///< which QC/PD/VOOC/etc (bits 3–0)
    };

    enum PowerLimit_t
    {
      PL_18W = 0,
      PL_24W = 1,
      PL_36W = 2,
      PL_60W = 3,
      PL_COUNT
    };

    /**
     * @brief Turn a PowerLimit_t into a human string ("18W", etc.)
     */
    static inline const char *powerLimitToString(PowerLimit_t lim)
    {
      static const char *names[PL_COUNT] = {
          "18W",
          "24W",
          "36W",
          "60W"};
      return (lim < PL_COUNT) ? names[lim] : "Unknown";
    }

    enum PDCmd_t
    {
      HARDRESET = 1
    };

    enum QuickChargeConfig
    {
      QC_CONF_NONE = 0,
      QC_CONF_PE = BIT(0),
      QC_CONF_SCP = BIT(2),
      QC_CONF_FCP = BIT(3),
      QC_CONF_QC = BIT(4),
      QC_CONF_PD = BIT(5),
      QC_CONF_PORT2 = BIT(6),
      QC_CONF_PORT1 = BIT(7),
      QC_CONF_AFC = BIT(8 + 6),
      QC_CONF_SFCP = BIT(8 + 7),
      QC_CONF_ALL = QC_CONF_PE | QC_CONF_SCP | QC_CONF_FCP | QC_CONF_QC | QC_CONF_PD |
                    QC_CONF_PORT1 | QC_CONF_PORT2 | QC_CONF_AFC | QC_CONF_SFCP
    };

    enum QuickChargePowerClass
    {
      QC_PWR_9V,
      QC_PWR_12V,
      QC_PWR_20V_1,
      QC_PWR_20V_2
    };

  private:
    enum ADCDataType
    {
      ADC_VIN = 1,
      ADC_VOUT = 2,
      ADC_IOUT_USB_C = 3,
      ADC_IOUT_USB_A = 4,
      ADC_TEMPERATURE = 6,
    };

    I2CInterface &_i2c;
    // TwoWire &_i2c;

    int i2cReadReg8(const uint8_t reg);
    int i2cWriteReg8(const uint8_t reg, const uint8_t data);

    void unlock_i2c_write();
    void lock_i2c_write();

    uint16_t readADCDataBuffer(const enum ADCDataType type);

  public:
    SW35xx(I2CInterface &i2c);
    // SW35xx(TwoWire &i2c = Wire);
    ~SW35xx();
    void begin();

    /**
     * @brief   Read bits [2:0] of REG 0x01 (chip version).
     * @return  0–7, or 0xFF on error.
     */
    uint8_t getChipVersion();

    /**
     * @brief Read REG 0x06: fast-charge indicator, PD version, and protocol.
     * @return a FastChargeInfo struct (all fields zeroed on I²C error)
     */
    FastChargeInfo getFastChargeInfo();

    /**
     * @brief Read PWR_CONF (Reg 0xA6) bits [1:0].
     */
    PowerLimit_t getPowerLimit();

    /**
     * @brief Write PWR_CONF (Reg 0xA6) bits [1:0].
     * @param lim one of PL_18W, PL_24W, PL_36W or PL_60W.
     */
    void setPowerLimit(PowerLimit_t lim);

    /**
     * @brief Read the current charging status
     */
    void readStatus(const bool useADCDataBuffer = false);
    /**
     * @brief Returns the voltage of the NTC in mV
     */
    float readTemperature(const bool useADCDataBuffer = false);
    /**
     * @brief Send PD command
     *
     * @note This chip seems to be able to send many kinds of PD commands, but the register documentation only has hardreset.
     * If you have a PD packet capture tool, you can try different parameters 2 to 15 to find out the corresponding command.
     */
    void sendPDCmd(PDCmd_t cmd);
    /**
     * @brief Rebroadcast PDO. After changing the maximum current, you need to call this function or replug the USB cable to make the setting take effect.
     */
    void rebroadcastPDO();
    /**
     * @brief Enable or disable the support for certain quick charge features
     * @param flags Multiple values of QuickChargeConfig combined with bitwise or
     */
    void setQuickChargeConfiguration(const uint16_t flags,
                                     const enum QuickChargePowerClass power);
    /**
     * @brief Set the current of all PD groups to 5A. If your chip is not sw3518s, please use it with caution.
     */
    void setMaxCurrent5A();
    /**
     * @brief Set the maximum output current for a fixed voltage group
     *
     * @param ma_xx The maximum output current of each group, unit is milliampere, the minimum scale is 50ma, set to 0 to turn it off
     * @note 5v cannot be turned off
     */
    void setMaxCurrentsFixed(uint32_t ma_5v, uint32_t ma_9v, uint32_t ma_12v, uint32_t ma_15v, uint32_t ma_20v);
    /**
     * @brief Set the maximum output current of the PPS group
     *
     * @param ma_xxx Maximum output current of each group, unit is milliampere, minimum scale is 50ma, set to 0 to turn off
     * @note Note that when the maximum power configured by the PD is greater than 60W, pps1 will not broadcast
     *  (TODO: the datasheet says so, I haven't tried it)
     *  The maximum voltage of pps1 needs to be greater than the maximum voltage of pps0, otherwise pps1 will not broadcast;
     */
    void setMaxCurrentsPPS(uint32_t ma_pps1, uint32_t ma_pps2);

    void resetPDLimits();

    /**
     * @brief Read the PWR_STATUS register (0x07) and parse the three control bits.
     * @return a PowerStatus struct with buckOn, port1On, port2On flags.
     */
    PowerStatus getPowerStatus();

    /**
    //  * @brief Reset maximum output current
    //  *
    //  * @note The current of the 20v group will not be reset
    //  */
    // void resetMaxCurrents();
    // /**
    //  * @brief Enable Emarker detection
    //  */
    // void enableEmarker();
    // /**
    //  * @brief Disable Emarker detection
    //  */
    // void disableEmarker();
  public:
    /**
     * @brief Input voltage
     */
    uint16_t vin_mV;
    /**
     * @brief Output voltage
     */
    uint16_t vout_mV;
    /**
     * @brief Output current 1(type-C)
     */
    uint16_t iout_usbc_mA;
    /**
     * @brief Output current 2(type-A)
     */
    uint16_t iout_usba_mA;
    /**
     * @brief Fast charging protocol
     */
    enum fastChargeType_t fastChargeType;
    /**
     * @brief PD version (2 or 3)
     */
    uint8_t PDVersion;

  public:
    // TODO

  private:
    bool _last_config_read_success;
  };

} // namespace SW35xx-lib
