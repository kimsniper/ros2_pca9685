#ifndef PCA9685_HAL_H
#define PCA9685_HAL_H

#include <cstdint>
#include <string>

/* Hardware Specific Components */
extern "C" {
#include <errno.h>
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
}

class Pca9685Hal {
public:
    Pca9685Hal(const std::string &device, const std::uint8_t i2c_address, const uint8_t i2c_gen_address);
    
    ~Pca9685Hal();

    typedef int16_t Pca9685_Error_t;

    static constexpr Pca9685_Error_t PCA9685_ERR    = -1;
    static constexpr Pca9685_Error_t PCA9685_OK     = 0;

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
    Pca9685_Error_t pca9685_i2c_hal_read(const std::uint8_t reg, std::uint8_t aRxBuffer[], const std::uint16_t count);

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
    Pca9685_Error_t pca9685_i2c_hal_read(const std::uint8_t reg, std::uint8_t &RxBuffer);

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
    Pca9685_Error_t pca9685_i2c_hal_write(const std::uint8_t reg, std::uint8_t aTxBuffer[], const std::uint16_t count);

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
    Pca9685_Error_t pca9685_i2c_hal_write(const std::uint8_t reg, std::uint8_t TxBuffer);

    /**
    * @brief        Execute I2C byte write.
    * @details      Execute I2C byte write sequence.
    *
    * @param[out]   aTxBuffer   Array to the data that will be written.
    *
    * @return       Pca9685_Error_t     Return code.
    *
    */
    Pca9685_Error_t pca9685_i2c_hal_write(std::uint8_t &aTxBuffer);

    /**
     * @brief        Execute ms delay.
     * @details      Execute ms delay for hal usage.
     *
     * @param[in]    ms      Time in milliseconds.
     *
     * @return       void
     *
     */
    void pca9685_i2c_hal_ms_delay(std::uint32_t ms);

 private:

    int fd_bus;
    int fd_bus_reset;

};


#endif  // PCA9685_HAL_H
