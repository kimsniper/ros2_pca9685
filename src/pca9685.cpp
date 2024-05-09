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
#include "ros2_pca9685/pca9685.h"

#include <iostream>
#include <cmath>

Pca9685::Pca9685(const std::string &device, int i2c_address, int i2c_gen_address)
{
    pca9685_i2c = std::make_unique<Pca9685Hal>(device, i2c_address, i2c_gen_address);

    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;

    /* Perform device reset */
    ret |= Pca9685_Reset();

    /* Enable register auto incremement */
    Pca9685_AutoIncrement(Pca9685::PCA9685_AUTOINCR_ON); /* Register increment every read/write */

    if(Pca9685Hal::PCA9685_OK == ret)
    {
        std::cout << "PCA9685 initialization successful" << std::endl;
    }
    else
    {
        std::cerr << "PCA9685 initialization failed!" << std::endl;
    }
}

Pca9685::~Pca9685()
{
    /* Perform device reset */
    Pca9685_Reset();
}

/*==================================================================================================
*                                       PRIVATE MEMBER FUNCTIONS
==================================================================================================*/

Pca9685Hal::Pca9685_Error_t Pca9685::Pca9685_ReadMode1(std::uint8_t &mode)
{
    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;
    const std::uint8_t reg = REG_MODE_1;
    std::uint8_t data = 0U;

    ret |= pca9685_i2c->pca9685_i2c_hal_read(reg, data);

    if (Pca9685Hal::PCA9685_OK == ret)
    {
        mode = data;
    }

    return ret;
}

Pca9685Hal::Pca9685_Error_t Pca9685::Pca9685_ReadMode2(std::uint8_t &mode)
{
    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;
    const std::uint8_t reg = REG_MODE_2;
    std::uint8_t data = 0U;

    ret |= pca9685_i2c->pca9685_i2c_hal_read(reg, data);

    if (Pca9685Hal::PCA9685_OK == ret)
    {
        mode = data;
    }

    return ret;
}

/*==================================================================================================
*                                       PUBLIC MEMBER FUNCTIONS
==================================================================================================*/

Pca9685Hal::Pca9685_Error_t Pca9685::Pca9685_clock(const Pca9685_AutoExtClk_t clk)
{
    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;
    const std::uint8_t reg = REG_MODE_1;
    std::uint8_t mode {0};
    std::uint8_t data {0};

    /* Parameter check */
    if(clk > PCA9685_CLK_EXTERNAL)
    {
        ret |= Pca9685Hal::PCA9685_ERR;
    }
    else
    {
        ret |= Pca9685_ReadMode1(mode);

        if(Pca9685Hal::PCA9685_OK == ret)
        {
            data = (mode & 0xBF) | (clk << 6);
            ret |= pca9685_i2c->pca9685_i2c_hal_write(reg, data);
        }
        
    }

    return ret;
}

Pca9685Hal::Pca9685_Error_t Pca9685::Pca9685_AutoIncrement(const pca9685_AutoIncr_t setting)
{
    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;
    const std::uint8_t reg = REG_MODE_1;
    std::uint8_t mode {0};
    std::uint8_t data {0};

    /* Parameter check */
    if(setting > PCA9685_AUTOINCR_ON)
    {
        ret |= Pca9685Hal::PCA9685_ERR;
    }
    else
    {
        ret |= Pca9685_ReadMode1(mode);

        if(Pca9685Hal::PCA9685_OK == ret)
        {
            data = (mode & 0xDF) | (setting << 5);
            ret |= pca9685_i2c->pca9685_i2c_hal_write(reg, data);
        }
        
    }

    return ret;
}

Pca9685Hal::Pca9685_Error_t Pca9685::Pca9685_Restart(void)
{
    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;
    const std::uint8_t reg = REG_MODE_1;
    std::uint8_t data {0};
    std::uint8_t mode {0};
    if(Pca9685_ReadMode1(mode) != Pca9685Hal::PCA9685_OK && !(mode & (1 << 7)))
        return Pca9685Hal::PCA9685_ERR;
    
    data = mode | (mode & ~(1 << 4));
    ret |= pca9685_i2c->pca9685_i2c_hal_write(reg, data);
    pca9685_i2c->pca9685_i2c_hal_ms_delay(STAB_TIME);
    if(Pca9685_ReadMode1(mode) != Pca9685Hal::PCA9685_OK)
        return Pca9685Hal::PCA9685_ERR;
    data = mode | (mode & (1 << 7));
    ret |= pca9685_i2c->pca9685_i2c_hal_write(reg, data);
    return ret;
}

Pca9685Hal::Pca9685_Error_t Pca9685::Pca9685_PwrMode(const Pca9685_PwrMode_t pwr_mode)
{
    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;
    const std::uint8_t reg = REG_MODE_1;
    std::uint8_t mode {0};
    std::uint8_t data {0};

    /* Parameter check */
    if(pwr_mode > PCA9685_MODE_SLEEP)
    {
        ret |= Pca9685Hal::PCA9685_ERR;
    }
    else
    {
        ret |= Pca9685_ReadMode1(mode);

        if(Pca9685Hal::PCA9685_OK == ret)
        {
            data = (mode & 0xEF) | (pwr_mode << 4);
            ret |= pca9685_i2c->pca9685_i2c_hal_write(reg, data);
            pca9685_i2c->pca9685_i2c_hal_ms_delay(STAB_TIME);
        }
    }

    return ret;
}

Pca9685Hal::Pca9685_Error_t Pca9685::Pca9685_Reset(void)
{
    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;
    std::uint8_t data = SWRST;

    ret |= pca9685_i2c->pca9685_i2c_hal_write(data);

    pca9685_i2c->pca9685_i2c_hal_ms_delay(STAB_TIME);

    return ret;
}

Pca9685Hal::Pca9685_Error_t Pca9685::Pca9685_OutputInit(const pca9685_OutputSet_t setting)
{
    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;
    const std::uint8_t reg = REG_MODE_2;
    std::uint8_t mode {0};
    std::uint8_t data {0};

    ret |= Pca9685_ReadMode2(mode);

    if(Pca9685Hal::PCA9685_OK == ret)
    {
        data = (mode & 0xE0) | (setting.invrt << 4) | (setting.och << 3) | (setting.outdrv << 2) | (setting.outne << 1);
        ret |= pca9685_i2c->pca9685_i2c_hal_write(reg, data);
    }

    return ret;
}

Pca9685Hal::Pca9685_Error_t Pca9685::Pca9685_LedSet(const std::uint8_t led_no, Pca9685_LedState_t state)
{
    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;
    const std::uint8_t reg = (led_no * 4) + LED_OFFSET_ADR;
    std::uint8_t data[4];

    /* Parameter check */
    if((state > PCA9685_LED_ON) || (led_no > MAX_CHANNEL_NUM))
    {
        ret |= Pca9685Hal::PCA9685_ERR;
    }
    else
    {
        data[1] = 1 << 4;
        if (state == PCA9685_LED_OFF)
        {
            data[3] = 1 << 4;
        }
            
        ret |= pca9685_i2c->pca9685_i2c_hal_write(reg, data, 4);
        
    }

    return ret;
}

Pca9685Hal::Pca9685_Error_t Pca9685::Pca9685_AllLedSet(const Pca9685_LedState_t state)
{
    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;
    const std::uint8_t reg = REG_ALL_LED;
    std::uint8_t data[4];

    /* Parameter check */
    if(state > PCA9685_LED_ON)
    {
        ret |= Pca9685Hal::PCA9685_ERR;
    }
    else
    {
        data[1] = 1 << 4;
        if (state == PCA9685_LED_OFF)
        {
            data[3] = 1 << 4;
        }
            
        ret |= pca9685_i2c->pca9685_i2c_hal_write(reg, data, 4);
        
    }

    return ret;
}

Pca9685Hal::Pca9685_Error_t Pca9685::Pca9685_LedPwmSet(const std::uint8_t led_no, const double d_cycle, const double delay)
{
    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;
    const std::uint8_t reg = (led_no * 4) + LED_OFFSET_ADR;
    std::uint8_t data[4];
    std::uint16_t delay_tm = round(delay * .01f * PWM_OUTPUT_COUNTER_MAX);
    std::uint16_t led_on_tm = round(d_cycle * .01f * PWM_OUTPUT_COUNTER_MAX);
    std::uint16_t led_off_tm = delay_tm + led_on_tm;

    /* Parameter check */
    if((led_no > MAX_CHANNEL_NUM) || (d_cycle > MAX_DUTYCYCLE_VAL))
    {
        ret |= Pca9685Hal::PCA9685_ERR;
    }
    else
    {
        if(delay_tm == 0)
        {
            delay_tm = 1;
        }

        data[0] = (delay_tm - 1) & 0xFF;
        data[1] = (delay_tm - 1) >> 8;
        data[2] = (led_off_tm > PWM_OUTPUT_COUNTER_MAX) ? (PWM_OUTPUT_COUNTER_MAX - led_off_tm) & 0xFF : (led_off_tm - 1) & 0xFF;
        data[3] = (led_off_tm > PWM_OUTPUT_COUNTER_MAX) ? (PWM_OUTPUT_COUNTER_MAX - led_off_tm) >> 8 : (led_off_tm - 1) >> 8;

        ret = pca9685_i2c->pca9685_i2c_hal_write(reg, data, 4);
        
    }

    return ret;
}

Pca9685Hal::Pca9685_Error_t Pca9685::Pca9685_AllLedPwmSet(const double d_cycle, const double delay)
{
    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;
    const std::uint8_t reg = REG_ALL_LED;
    std::uint8_t data[4];
    std::uint16_t delay_tm = round(delay * .01f * PWM_OUTPUT_COUNTER_MAX);
    std::uint16_t led_on_tm = round(d_cycle * .01f * PWM_OUTPUT_COUNTER_MAX);
    std::uint16_t led_off_tm = delay_tm + led_on_tm;

    /* Parameter check */
    if(d_cycle > MAX_DUTYCYCLE_VAL)
    {
        ret |= Pca9685Hal::PCA9685_ERR;
    }
    else
    {
        if(delay_tm == 0)
        {
            delay_tm = 1;
        }

        data[0] = (delay_tm - 1) & 0xFF;
        data[1] = (delay_tm - 1) >> 8;
        data[2] = (led_off_tm > PWM_OUTPUT_COUNTER_MAX) ? (PWM_OUTPUT_COUNTER_MAX - led_off_tm) & 0xFF : (led_off_tm - 1) & 0xFF;
        data[3] = (led_off_tm > PWM_OUTPUT_COUNTER_MAX) ? (PWM_OUTPUT_COUNTER_MAX - led_off_tm) >> 8 : (led_off_tm - 1) >> 8;

        ret = pca9685_i2c->pca9685_i2c_hal_write(reg, data, 4);
        
    }

    return ret;
}

Pca9685Hal::Pca9685_Error_t Pca9685::Pca9685_SetPrescale(const double frequency, const double osc_clk_hz)
{
    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;
    const std::uint8_t reg = REG_PRE_SCALE;
    std::uint8_t data {0};

    /* Parameter check */
    if(frequency < 24 || frequency > 1526)
    {
        ret |= Pca9685Hal::PCA9685_ERR;
    }
    else
    {
        /* Ensure that device is in sleep mode when setting the prescaler */
        ret |= Pca9685_PwrMode(PCA9685_MODE_SLEEP);

        if(Pca9685Hal::PCA9685_OK == ret)
        {
            data = round(osc_clk_hz / (PWM_OUTPUT_COUNTER_MAX * frequency)) - 1;
            ret |= pca9685_i2c->pca9685_i2c_hal_write(reg, data);
        }
    }

    return ret;
}

Pca9685Hal::Pca9685_Error_t Pca9685::Pca9685_SetAllCallAddr(const std::uint8_t allcall_addr)
{
    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;
    const std::uint8_t reg = REG_ALLCALLADR;
    std::uint8_t data {0};

    /* Parameter check */
    if(allcall_addr > SUB_ALL_ADDR_MAX_VAL)
    {
        ret |= Pca9685Hal::PCA9685_ERR;
    }
    else
    {
        data = allcall_addr << SUB_ALL_ADDR_SHIFT;
        data &= SUB_ALL_ADDR_MASK;
        ret |= pca9685_i2c->pca9685_i2c_hal_write(reg, data);
    }

    return ret;
}

Pca9685Hal::Pca9685_Error_t Pca9685::Pca9685_SetSubAddr(const Pca9685_SubAddrNo_t addr_no, const std::uint8_t sub_addr)
{
    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;
    const std::uint8_t reg = static_cast<std::uint8_t>(addr_no);
    std::uint8_t data {0};

    /* Parameter check */
    if((addr_no > PCA9685_SUB_ADDR_3) || (sub_addr > SUB_ALL_ADDR_MAX_VAL))
    {
        ret |= Pca9685Hal::PCA9685_ERR;
    }
    else
    {
        data = sub_addr << SUB_ALL_ADDR_SHIFT;
        data &= SUB_ALL_ADDR_MASK;
        ret |= pca9685_i2c->pca9685_i2c_hal_write(reg, data);
    }

    return ret;
}

Pca9685Hal::Pca9685_Error_t Pca9685::Pca9685_SetSubAddrResp(Pca9685_SubAddrNo_t sub_addr, const Pca9685_AddrResp_t resp)
{
    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;
    const std::uint8_t reg = REG_MODE_1;
    std::uint8_t mode {0};
    std::uint8_t data {0};

    /* Parameter check */
    if((sub_addr > PCA9685_SUB_ADDR_3) || (resp > PCA9685_ADDR_RESPOND))
    {
        ret |= Pca9685Hal::PCA9685_ERR;
    }
    else
    {
        ret |= Pca9685_ReadMode1(mode);

        if(Pca9685Hal::PCA9685_OK == ret)
        {
            if(sub_addr == PCA9685_SUB_ADDR_1)
            {
                sub_addr = PCA9685_SUB_ADDR_3;
            }
                
            else if(sub_addr == PCA9685_SUB_ADDR_3) 
            {
                sub_addr = PCA9685_SUB_ADDR_1;
            }

            data = (mode & ~(1 << sub_addr)) | (resp << sub_addr);
            ret |= pca9685_i2c->pca9685_i2c_hal_write(reg, data);
        }
        
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Configures the DLPF setting.
* @details      Configures the digital low pass filter setting.
*
* @param[in]    eDlpfCfg    Digital low pass filter setting value.
*
* @return       Pca9685Hal::Pca9685_Error_t     Return code.
*
*/
Pca9685Hal::Pca9685_Error_t Pca9685::Pca9685_SetAllCallAddrResp(Pca9685_AddrResp_t resp)
{
    Pca9685Hal::Pca9685_Error_t ret = Pca9685Hal::PCA9685_OK;
    const std::uint8_t reg = REG_MODE_1;
    std::uint8_t mode {0};
    std::uint8_t data {0};

    /* Parameter check */
    if(resp > PCA9685_ADDR_RESPOND)
    {
        ret |= Pca9685Hal::PCA9685_ERR;
    }
    else
    {
        ret |= Pca9685_ReadMode1(mode);

        if(Pca9685Hal::PCA9685_OK == ret)
        {
            data = (mode & ~(1 << 0)) | resp;
            ret |= pca9685_i2c->pca9685_i2c_hal_write(reg, data);
        }
        
    }

    return ret;
}
