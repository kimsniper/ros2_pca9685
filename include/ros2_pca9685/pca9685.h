
/*
 * Copyright (c) 2022, Mezael Docoy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PCA9685_H
#define PCA9685_H

#include "ros2_pca9685/pca9685_hal.h"

#include <string>
#include <memory>
#include <cstdint>

class Pca9685 {
public:
    explicit Pca9685(const std::string &device = "/dev/i2c-1", int i2c_address = I2C_ADDRESS_PCA9685, int i2c_gen_address = I2C_GEN_CALL_ADDRESS_PCA9685);

    ~Pca9685();

    typedef enum{
        PCA9685_CLK_INTERNAL = 0x00,
        PCA9685_CLK_EXTERNAL  = 0x01,
    } Pca9685_AutoExtClk_t;

    typedef enum{
        PCA9685_AUTOINCR_OFF = 0x00,
        PCA9685_AUTOINCR_ON  = 0x01,
    } pca9685_AutoIncr_t;

    typedef enum{
        PCA9685_MODE_NORMAL = 0x00,
        PCA9685_MODE_SLEEP  = 0x01,
    } Pca9685_PwrMode_t;

    typedef enum{
        PCA9685_ADDR_NORESPOND = 0x00,
        PCA9685_ADDR_RESPOND = 0x01,
    } Pca9685_AddrResp_t;

    typedef enum{
        PCA9685_SUB_ADDR_1 = 0x02,
        PCA9685_SUB_ADDR_2 = 0x03,
        PCA9685_SUB_ADDR_3 = 0x04,
    } Pca9685_SubAddrNo_t;

    typedef enum{
        PCA9685_LED_OFF = 0x00,
        PCA9685_LED_ON  = 0x01,
    } Pca9685_LedState_t;

    typedef enum{
        PCA9685_OUTPUT_NOTINVERT = 0x00,
        PCA9685_OUTPUT_INVERT = 0x01,
    } Pca9685_OutputInvert_t;

    typedef enum{
        PCA9685_CH_ONSTOP = 0x00,
        PCA9685_CH_ONACK = 0x01,
    } Pca9685_OutputChange_t;

    typedef enum{
        PCA9685_OUTPUT_LOW = 0x00,
        PCA9685_OUTPUT_HIGH = 0x01,
        PCA9685_OUTPUT_HIGH_IMPEDANCE = 0x02,
    } Pca9685_OutputNotEnable_t;

    typedef enum{
        PCA9685_OUTPUT_OPEN_DRAIN = 0x00,
        PCA9685_OUTPUT_TOTEM_POLE = 0x01,
    } Pca9685_OutputDrive_t;

    typedef struct{
        Pca9685_OutputDrive_t outdrv;
        Pca9685_OutputNotEnable_t outne;
        Pca9685_OutputChange_t och;
        Pca9685_OutputInvert_t invrt;
    } pca9685_OutputSet_t;

    /*==================================================================================================
    *                                       FUNCTION Prototypes
    ==================================================================================================*/

    /**
    * @brief        Set power mode of the device.
    * @details      Set power mode of the device (sleep/cycle/normal).
    *
    * @param[in]    ePwrMode    Power mode.
    *
    * @return       Pca9685Hal::Pca9685_Error_t     Return code.
    *
    */
    Pca9685Hal::Pca9685_Error_t Pca9685_clock(const Pca9685_AutoExtClk_t clk);

    /**
     * @brief Set PCA9685 register auto increment in mode 1 register
     */
    Pca9685Hal::Pca9685_Error_t Pca9685_AutoIncrement(const pca9685_AutoIncr_t setting);

    /**
     * @brief PCA9685 restart
     */
    Pca9685Hal::Pca9685_Error_t Pca9685_Restart(void);

    /**
     * @brief PCA9685 sleep mode setting
     */
    Pca9685Hal::Pca9685_Error_t Pca9685_PwrMode(const Pca9685_PwrMode_t pwr_mode);

    /**
     * @brief PCA9685 reset
     */
    Pca9685Hal::Pca9685_Error_t Pca9685_Reset(void);

    /**
     * @brief PCA9685 output initialization
     */
    Pca9685Hal::Pca9685_Error_t Pca9685_OutputInit(const pca9685_OutputSet_t setting);

    /**
     * @brief Set PCA9685 LEDx HIGH/LOW output 
     */
    Pca9685Hal::Pca9685_Error_t Pca9685_LedSet(const std::uint8_t led_no, Pca9685_LedState_t state);

    /**
     * @brief Set PCA9685 all LEDs HIGH/LOW output 
     */
    Pca9685Hal::Pca9685_Error_t Pca9685_AllLedSet(const Pca9685_LedState_t state);

    /**
     * @brief Set PCA9685 LEDx PWM output
     */
    Pca9685Hal::Pca9685_Error_t Pca9685_LedPwmSet(const std::uint8_t led_no, const double d_cycle, const double delay=0);

    /**
     * @brief Set PCA9685 all LEDs PWM output
     */
    Pca9685Hal::Pca9685_Error_t Pca9685_AllLedPwmSet(const double d_cycle, const double delay = 0);

    /**
     * @brief Set PCA9685 pre scale settings
     */
    Pca9685Hal::Pca9685_Error_t Pca9685_SetPrescale(const double frequency, const double osc_clk_hz = INTERNAL_OSC);

    /**
     * @brief Set PCA9685 all call address
     */
    Pca9685Hal::Pca9685_Error_t Pca9685_SetAllCallAddr(const std::uint8_t allcall_addr);

    /**
     * @brief Set PCA9685 sub address
     */
    Pca9685Hal::Pca9685_Error_t Pca9685_SetSubAddr(const Pca9685_SubAddrNo_t addr_no, const std::uint8_t sub_addr);

    /**
     * @brief Set PCA9685 sub address response type
     */
    Pca9685Hal::Pca9685_Error_t Pca9685_SetSubAddrResp(Pca9685_SubAddrNo_t sub_addr, const Pca9685_AddrResp_t resp);

    /**
     * @brief Set PCA9685 all call address response type
     */
    Pca9685Hal::Pca9685_Error_t Pca9685_SetAllCallAddrResp(Pca9685_AddrResp_t resp);

private:
    Pca9685Hal::Pca9685_Error_t Pca9685_ReadMode1(std::uint8_t &mode);

    Pca9685Hal::Pca9685_Error_t Pca9685_ReadMode2(std::uint8_t &mode);

    std::unique_ptr<Pca9685Hal> pca9685_i2c;

    /**
     * @brief PCA9685 I2C slave addresses
     */
    static constexpr std::uint8_t I2C_ADDRESS_PCA9685             = 0x40;

    /**
     * @brief PCA9685 default addresses
     */
    static constexpr std::uint8_t I2C_GEN_CALL_ADDRESS_PCA9685    = 0x00;
    static constexpr std::uint8_t I2C_ALL_CALL_ADDRESS_PCA9685    = 0x70;
    static constexpr std::uint8_t I2C_SUB_ADDRESS_1_PCA9685       = 0x71;
    static constexpr std::uint8_t I2C_SUB_ADDRESS_2_PCA9685       = 0x72;
    static constexpr std::uint8_t I2C_SUB_ADDRESS_3_PCA9685       = 0x74;

    /**
     * @brief PCA9685 R/W Command registers
     */
    static constexpr std::uint8_t REG_RESET                       = 0x00;
    static constexpr std::uint8_t REG_MODE_1                      = 0x00;
    static constexpr std::uint8_t REG_MODE_2                      = 0x01;
    static constexpr std::uint8_t REG_ALLCALLADR                  = 0x05;
    static constexpr std::uint8_t REG_ALL_LED                     = 0xFA;
    static constexpr std::uint8_t REG_PRE_SCALE                   = 0xFE;
    static constexpr std::uint8_t REG_TEST_MODE                   = 0xFF;

    /**
     * @brief PCA9685 Mask/Shift values
     */
    static constexpr std::uint8_t SLEEP_MASK                      = 0x10;
    static constexpr std::uint8_t SUB_ALL_ADDR_MASK               = 0xFE;
    static constexpr std::uint8_t SUB_ALL_ADDR_SHIFT              = 0x01;

    /**
     * @brief PCA9685 software reset command
     */
    static constexpr std::uint8_t SWRST                           = 0x06;

    /**
     * @brief Other PCA9685 macros
     */
    static constexpr std::uint8_t LED_OFFSET_ADR                  = 0x06;
    static constexpr std::uint8_t STAB_TIME                       = 1;          /* Stabilization time (ms) */
    static constexpr std::uint16_t PWM_OUTPUT_COUNTER_MAX         = 0x1000;     /* 0000h to 0FFFh (12 bit) counter */
    static constexpr std::uint8_t SUB_ALL_ADDR_MAX_VAL            = 0xFE;
    static constexpr std::uint8_t MAX_CHANNEL_NUM                 = 15U;
    static constexpr std::uint8_t MAX_DUTYCYCLE_VAL               = 100U;

    static constexpr double INTERNAL_OSC                          = 25000000;   /*Internal oscillator value*/
};

#endif  // PCA9685_H
