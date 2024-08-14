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
 * @return A pointer to the initialized I2C bus, or NULL on failure.
 */
void * i2c_init(w_i2c_config_t *params);

err_t i2c_add_master_device(uint8_t is, uint16_t dev_addr, uint32_t cl_speed, void* bus_handle,void *dev);
/**
 * @brief Reads multiple bytes from an I2C slave device.
 *
 * Reads the specified number of bytes from the given I2C slave device.
 *
 * @param dev_addr The 7-bit address of the slave device.
 * @param length The number of bytes to read.
 * @param data Pointer to a buffer to store the read data.
 * @param timeout Maximum time in milliseconds to wait for the read operation.
 *
 * @return OK on success, or an error code on failure.
 */

err_t i2c_read(void * dev_handler, uint8_t reg_addr,  uint8_t length, uint8_t *data);

/**
 * @brief Writes multiples bytes from an I2C slave device.
 *
 * Writes multiples bytes of data from the specified I2C slave device.
 *
 * @param dev_addr The 7-bit address of the slave device.
 * @param data Pointer to a buffer with the data to send on the bus.
 * @param length Size, in bytes, of the write buffer.
 * @param timeout Maximum time in milliseconds to wait for the write operation.
 *
 * @return OK on success, or an error code on failure.
 */
err_t i2c_write(void * dev_handler, uint8_t reg_addr, uint8_t *data, uint8_t length);

err_t i2c_del_master_device(void * dev_handler);
err_t i2c_deinit(void * bus_handler);
#endif