#include "ros2_pca9685/pca9685_hal.h"

/* User header files */
#include <iostream>

Pca9685Hal::Pca9685Hal(const std::string& device, const uint8_t i2c_address, const uint8_t i2c_gen_address) 
{ 
    /* Communicating with main address */
    fd_bus = open(device.c_str(), O_RDWR);
    if (fd_bus < 0)
    {
        std::cerr << "Failed to open file descriptor!" << std::endl;
        exit(1);
    }
    if (ioctl(fd_bus, I2C_SLAVE, i2c_address) < 0) 
    {
        std::cerr << "I2c communication failed" << std::endl;
        exit(1);
    }

    /* Communicating with gen call address */
    fd_bus_reset = open(device.c_str(), O_RDWR);
    if (fd_bus_reset < 0)
    {
        std::cerr << "Failed to open file descriptor!" << std::endl;
        exit(1);
    }
    if (ioctl(fd_bus_reset, I2C_SLAVE, i2c_gen_address) < 0) 
    {
        std::cerr << "I2c communication failed" << std::endl;
        exit(1);
    }

    std::cerr << "I2c communication started. ." << std::endl;
}

Pca9685Hal::~Pca9685Hal() { 
    close(fd_bus); 
    close(fd_bus_reset); 
}

/*==================================================================================================
*                                       PUBLIC FUNCTIONS
==================================================================================================*/

/*================================================================================================*/
/**
* @brief        Execute I2C read.
* @details      Execute I2C read sequence.
*
* @param[in]    reg         Register address.
* @param[in]    count       Number of bytes to read.
* @param[out]   aRxBuffer   Array to which data will be stored.
*
* @return       Pca9685_Error_t     Return code.
*
*/
Pca9685Hal::Pca9685_Error_t Pca9685Hal::pca9685_i2c_hal_read(const std::uint8_t reg, std::uint8_t aRxBuffer[], const std::uint16_t count)
{
    /* Param checking */
    if(count == 0U)
    {
        return PCA9685_ERR;
    }
    else
    {
        const auto i2c_ret = i2c_smbus_read_i2c_block_data(fd_bus, reg, count, aRxBuffer);
        if(i2c_ret < 0)
        {
            return PCA9685_ERR;
        }
    }

    return PCA9685_OK;
}

/*================================================================================================*/
/**
 * @brief        Execute I2C read.
 * @details      Execute I2C read sequence.
 *
 * @param[in]    reg         Register address.
 * @param[out]   RxBuffer    Reference to which data will be stored.
 *
 * @return       Pca9685_Error_t     Return code.
 *
 */
Pca9685Hal::Pca9685_Error_t Pca9685Hal::pca9685_i2c_hal_read(const std::uint8_t reg, std::uint8_t &RxBuffer)
{
    std::uint8_t RxBuffer_ {0};

    const auto i2c_ret = i2c_smbus_read_i2c_block_data(fd_bus, reg, 1U, &RxBuffer_);
    if(i2c_ret < 0)
    {
        return PCA9685_ERR;
    }

    RxBuffer = RxBuffer_;

    return PCA9685_OK;
}

/*================================================================================================*/
/**
* @brief        Execute I2C write.
* @details      Execute I2C write sequence.
*
* @param[in]    reg         Register address.
* @param[in]    count       Number of bytes to read.
* @param[out]   aTxBuffer   Array to the data that will be written.
*
* @return       Pca9685_Error_t     Return code.
*
*/
Pca9685Hal::Pca9685_Error_t Pca9685Hal::pca9685_i2c_hal_write(const std::uint8_t reg, std::uint8_t aTxBuffer[], const std::uint16_t count)
{
    if(count == 0U)
    {
        const auto i2c_ret = i2c_smbus_write_byte(fd_bus, aTxBuffer[0]);
        if(i2c_ret < 0)
        {
            return PCA9685_ERR;
        }
    }
    else
    {
        const auto i2c_ret = i2c_smbus_write_i2c_block_data(fd_bus, reg, count, aTxBuffer);
        if(i2c_ret < 0)
        {
            return PCA9685_ERR;
        }
    }

    return PCA9685_OK;
}

/*================================================================================================*/
/**
 * @brief        Execute I2C write.
 * @details      Execute I2C write sequence.
 *
 * @param[in]    reg         Register address.
 * @param[out]   TxBuffer    Data that will be written.
 *
 * @return       Pca9685_Error_t     Return code.
 *
 */
Pca9685Hal::Pca9685_Error_t Pca9685Hal::pca9685_i2c_hal_write(const std::uint8_t reg, std::uint8_t TxBuffer)
{
    std::uint8_t TxBuffer_ = TxBuffer;

    const auto i2c_ret = i2c_smbus_write_i2c_block_data(fd_bus, reg, 1U, &TxBuffer_);
    if(i2c_ret < 0)
    {
        return PCA9685_ERR;
    }

    TxBuffer = TxBuffer_;

    return PCA9685_OK;
}

/*================================================================================================*/
/**
* @brief        Execute I2C byte write.
* @details      Execute I2C byte write sequence.
*
* @param[out]   aTxBuffer   Array to the data that will be written.
*
* @return       Pca9685_Error_t     Return code.
*
*/
Pca9685Hal::Pca9685_Error_t Pca9685Hal::pca9685_i2c_hal_write(std::uint8_t &aTxBuffer)
{
    const auto i2c_ret = i2c_smbus_write_byte(fd_bus_reset, aTxBuffer);
    if(i2c_ret < 0)
    {
        return PCA9685_ERR;
    }

    return PCA9685_OK;
}

/*================================================================================================*/
/**
* @brief        Execute ms delay.
* @details      Execute ms delay for hal usage.
*
* @param[in]    ms      Time in milliseconds.
*
* @return       void
*
*/
void Pca9685Hal::pca9685_i2c_hal_ms_delay(std::uint32_t ms) {
    
    usleep(ms * 1000);
    
}
