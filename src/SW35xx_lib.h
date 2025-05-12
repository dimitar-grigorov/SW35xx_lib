#pragma once
#include <stdint.h>

#include "I2CInterface.h"
// #include <Wire.h>

#ifndef BIT
#define BIT(x) (1 << (x))
#endif

namespace SW35xx_lib
{
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

    struct SwitchStatus
    {
      bool buckOn;  ///< Bit 2: buck switch (1 = on, 0 = off)
      bool portCOn; ///< Bit 0: C-port switch (1 = on, 0 = off)
      bool portAOn; ///< Bit 1: A-port switch (1 = on, 0 = off)
    };

    /// Decoded meanings for REG 0x08 bits 7–4.
    /// Note: Port A is considered “connected” if current > 80 mA,
    /// and “disconnected” if current drops below 15 mA.
    enum class PresenceStatus : uint8_t
    {
      AA_None = 1,   ///< AA mode: neither A1 nor A2 has a device
      AA_A1 = 2,     ///< AA mode: only A1 has a device
      AA_A2 = 3,     ///< AA mode: only A2 has a device
      AA_Both = 4,   ///< AA mode: both A1 & A2 have devices
      AC_None = 5,   ///< AC mode: neither A nor C has a device
      AC_C = 6,      ///< AC mode: only C has a device
      AC_A = 7,      ///< AC mode: only A has a device
      AC_Both = 8,   ///< AC mode: both A & C have devices
      Unknown = 0xFF ///< any other code
    };

    static inline const char *presenceStatusToString(PresenceStatus s)
    {
      // index 0 = "Unknown", 1..8 = the codes above
      static const char *names[] = {
          "Unknown",
          "AA: none",
          "AA: only A1",
          "AA: only A2",
          "AA: A1 & A2",
          "AC: none",
          "AC: only C",
          "AC: only A",
          "AC: A & C"};
      uint8_t idx = (uint8_t)s;
      // idx runs 0..8, names has 9 entries
      return (idx < (sizeof(names) / sizeof(names[0])))
                 ? names[idx]
                 : names[0];
    }

    /// Which temperature to report under PPS/SCP: real NTC or fixed 45 °C?
    enum ADCVinTempSource_t
    {
      ADCVTS_NTC = 0, ///< report actual NTC temperature
      ADCVTS_45C = 1  ///< report a constant 45 °C
    };

    /**
     * @brief Non-PD power-limit selections (REG 0xA6 bits 1–0).
     *
     * Value | Power | VOUT ≤ 7 V | 7 V < VOUT ≤ 10 V | 10 V < VOUT ≤ 16 V | VOUT > 16 V
     * ------|-------|------------|-------------------|--------------------|-----------
     * 0     | 18 W  | 5 V @ 3.2 A | 9 V @ 2.2 A       | 12 V @ 1.7 A       | 20 V @ 1.4 A
     * 1     | 24 W  | 5 V @ 3.2 A | 9 V @ 3.2 A       | 12 V @ 2.2 A       | 20 V @ 1.4 A
     * 2     | 36 W  | 5 V @ 3.2 A | 9 V @ 3.2 A       | 12 V @ 3.2 A       | 20 V @ 2.2 A
     * 3     | 60 W  | 5 V @ 3.2 A | 9 V @ 3.2 A       | 12 V @ 3.2 A       | 20 V @ 3.2 A
     */
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

    /// Which port(s) the chip drives (REG 0xAB bits 3–2)
    enum PortConfig_t
    {
      PORT_SINGLE_A = 0, ///< single Type-A port
      PORT_DUAL_A = 1,   ///< two Type-A ports
      PORT_SINGLE_C = 2, ///< single Type-C port
      PORT_AC = 3        ///< one A and one C port (AC mode)
    };

    /**
     * @brief Convert a PortConfig_t to a human string.
     */
    static inline const char *portConfigToString(PortConfig_t cfg)
    {
      static const char *names[] = {
          "Single A",
          "Dual A",
          "Single C",
          "AC (A+C)"};
      return names[(uint8_t)cfg & 0x03];
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
     * @brief Read the PWR_STATUS register (0x07) and decode buck/C/A-port on/off.
     * @return SystemStatus with each flag true = switch closed/on.
     */
    SwitchStatus getSwitchStatus();

    /**
     * @brief Read REG 0x08 ("System status 1") and decode which ports see a connected device.
     * @return A PresenceStatus enum telling you which pattern was seen.
     */
    PresenceStatus getPresenceStatus();

    /**
     * @brief  Enable I²C write access to the extended PD_CONF (0xB0–0xBF) registers.
     *
     * According to REG 0x12 (I²C Enable control), you must write 0x20, then 0x40, then 0x80
     * to REG 0x12 before modifying any 0xB0–0xBF registers.
     */
    void enableI2CWrite();

    /**
     * @brief  Disable I²C write access (lock out further writes to 0xB0–0xBF).
     *
     * Writing 0x00 to REG 0x12 clears bits 7–5 and re-locks the extended write window.
     */
    void disableI2CWrite();

    /**
     * @brief Read REG 0x13 (“ADC Vin enable”).
     * @return true if ADC Vin measurement is enabled (bit 1 = 1).
     */
    bool isVinAdcEnabled();

    /**
     * @brief Enable or disable ADC Vin measurement.
     * @param enable true = turn on Vin ADC (bit 1=1), false = disable (bit 1=0).
     */
    void enableVinAdc(bool enable);

    /**
     * @brief Read REG 0x13 bit 6: whether to report NTC or fixed 45 °C.
     * @return ADCVTS_NTC or ADCVTS_45C.
     */
    ADCVinTempSource_t getVinTempSource();

    /**
     * @brief Set REG 0x13 bit 6: choose report of NTC vs 45 °C under PPS/SCP.
     * @param src ADCVTS_NTC → real NTC; ADCVTS_45C → fixed 45 °C.
     */
    void setVinTempSource(ADCVinTempSource_t src);

    /**
     * @brief Read PWR_CONF (Reg 0xA6) bits [1:0].
     */
    PowerLimit_t getPowerLimit();

    /**
     * @brief Write PWR_CONF (Reg 0xA6) bits [1:0].
     * Note: Non-PD power-limit selections!
     * @param lim one of PL_18W, PL_24W, PL_36W or PL_60W.
     */
    void setPowerLimit(PowerLimit_t lim);

    /**
     * @brief Read REG 0xAA bit 6: QC3.0 enable.
     * @return true if QC3.0 is enabled (bit 6 = 1).
     */
    bool isQc3Enabled();

    /**
     * @brief Enable or disable QC3.0 support.
     * @param enable true = set bit 6 to 1 (enable QC3.0), false = clear bit 6.
     */
    void enableQc3(bool enable);

    /**
     * @brief Read REG 0xAB bits 3–2 and return the current port config.
     */
    PortConfig_t getPortConfig();

    /**
     * @brief Write REG 0xAB bits 3–2 to select the port configuration.
     * @param cfg the desired PortConfig_t
     */
    void setPortConfig(PortConfig_t cfg);

    /**
     * @brief Read REG 0xAD bit 2: Samsung 1.2 V mode enable.
     * @return true if Samsung 1.2 V mode is enabled (bit 2 = 1).
     */
    bool isSamsung12VModeEnabled();

    /**
     * @brief Enable or disable Samsung 1.2 V mode (REG 0xAD bit 2).
     * @param enable true = set bit 2 to 1, false = clear bit 2.
     */
    void enableSamsung12VMode(bool enable);

    /**
     * @brief Read REG 0xAF: vendor ID high byte (VID[15:8]).
     * @return the high-byte of the USB-PD Vendor ID.
     */
    uint8_t getVidHigh();

    /**
     * @brief Write REG 0xAF: vendor ID high byte (VID[15:8]).
     * @param vidHigh the high-byte of the USB-PD Vendor ID to set.
     */
    void setVidHigh(uint8_t vidHigh);

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

  private:
    bool _last_config_read_success;
  };

} // namespace SW35xx-lib
