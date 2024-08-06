#ifndef _I2C_MODULE_H_
#define _I2C_MODULE_H_

#include <stdint.h>
#include "logs.h"


typedef struct 
{
    int sda_num;      
    int scl_num; 
    uint32_t clk_speed;     

} i2c_config_t;

typedef void* i2c_bus_t;

i2c_bus_t * i2c_init(i2c_config_t *params);
err_t i2c_read_byte();
err_t i2c_read();
err_t i2c_write_byte();
err_t i2c_write();

#endif