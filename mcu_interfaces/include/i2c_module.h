#ifndef _I2C_MODULE_H_
#define _I2C_MODULE_H_

#include <stdint.h>
#include "error_module.h"


typedef struct w_i2c_config_t
{
    int sda_num;      
    int scl_num; 
    uint32_t clk_speed;    

} w_i2c_config_t;

typedef void* i2c_bus_t;
typedef void* i2c_dev_t;


 /**
 * @brief Initializes an I2C bus in master mode.
 *
 * This function configures the specified I2C pins with pull-up resistors and initializes
 * the I2C driver in master mode. The provided clock speed is set for the I2C bus.
 *
 * @param params Pointer to a structure containing I2C configuration parameters.
 *   - params->sda_num: SDA pin number.
 *   - params->scl_num: SCL pin number.
 *   - params->clk_speed: I2C clock speed in Hz.
 * @return A pointer to the initialized I2C bus handle, or NULL on error.
 */
i2c_bus_t * i2c_init_bus(w_i2c_config_t *params);

/**
 * Adds a master device to an I2C bus.
 *
 * This function adds a master device to the specified I2C bus.
 *
 * @param is_seven Indicates whether to use a 7-bit or 10-bit device address.
 * @param dev_addr The device address.
 * @param cl_speed The clock speed in Hz.
 * @param bus_handle A pointer to the I2C bus handle.
 * @param dev A pointer to store the device handle.
 * @return E_OK on success, error code otherwise.
 */
err_t i2c_add_master_device(uint8_t is, uint16_t dev_addr, uint32_t cl_speed, void* bus_handle,void *dev);

/**
 * Reads data from an I2C device register.
 *
 * This function reads a specified number of bytes from a register on the given I2C device.
 *
 * @param dev_handler A pointer to the device handle.
 * @param reg_addr The register address to read from.
 * @param length The number of bytes to read.
 * @param data A pointer to the buffer where the read data will be stored.
 * @return E_OK on success, error code otherwise.
 *
 * @note The function currently overwrites the input data pointer with the internal buffer address.
 *       Consider using a separate buffer for storing the read data to avoid unexpected behavior.
 */
err_t i2c_read(void * dev_handler, uint8_t reg_addr,  uint8_t length, uint8_t *data);


/**
 * Writes data to an I2C device register.
 *
 * This function writes a specified number of bytes to a register on the given I2C device.
 *
 * @param dev_handler A pointer to the device handle.
 * @param reg_addr The register address to write to.
 * @param data The data to be written.
 * @param length The number of bytes to write.
 * @return E_OK on success, error code otherwise.
 */
err_t i2c_write(void * dev_handler, uint8_t reg_addr, uint8_t *data, uint8_t length);


/**
 * Deletes a master device from an I2C bus.
 *
 * This function removes a master device from the specified I2C bus.
 *
 * @param dev_handler A pointer to the device handle.
 * @return E_OK on success, error code otherwise.
 */
err_t i2c_del_master_device(void * dev_handler);

/**
 * Deinitializes an I2C bus. Is not threads safe.
 *
 * This function deinitializes the specified I2C bus.
 *
 * @param bus_handler A pointer to the bus handle.
 * @return E_OK on success, error code otherwise.
 */
err_t i2c_deinit(void * bus_handler);
#endif