#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "i2c_module.h"
#include "error_module.h"
#include <string.h>

#ifndef CONFIG_FREERTOS_HZ
#define CONFIG_FREERTOS_HZ 100  
#endif

#define I2C_NUM I2C_NUM_0
#define NULL_I2C_MEM_ADDR 0xFF   
#define I2C_ACK_CHECK_EN 0x1   



i2c_bus_t * i2c_init_bus(const w_i2c_config_t *params)
{
    i2c_master_bus_config_t conf ;
    conf.i2c_port = -1; //autoconfig
    conf.clk_source = I2C_CLK_SRC_DEFAULT;
    conf.sda_io_num = params->sda_num;
    conf.scl_io_num = params->scl_num;
    conf.glitch_ignore_cnt = 7;
    conf.flags.enable_internal_pullup = true;
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&conf, &bus_handle));
    return (i2c_bus_t *)bus_handle;  
};


err_t i2c_add_master_device(uint8_t is_seven, uint16_t dev_addr, uint32_t cl_speed, i2c_bus_t* bus_handle, i2c_dev_t *dev)
{
    i2c_device_config_t cfg;
    cfg.dev_addr_length = (is_seven!=0) ? I2C_ADDR_BIT_LEN_7 : I2C_ADDR_BIT_LEN_10;
    cfg.device_address = dev_addr;
    cfg.scl_speed_hz = cl_speed;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_handle_t * bus = (i2c_master_bus_handle_t *) bus_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &cfg, &dev_handle));
    i2c_dev_t* dev = &dev_handle;
    return E_OK;
};



err_t i2c_write(void * dev_handler, uint8_t reg_addr, const uint8_t *data, uint8_t length)
{
    uint8_t reg_and_data[length + 1];
    reg_and_data[0] = reg_addr;
    memcpy(&(reg_and_data[1]), data, length);
    i2c_master_dev_handle_t dev = *(i2c_master_dev_handle_t *)dev_handler;
    ESP_ERROR_CHECK(i2c_master_transmit(dev, reg_and_data, length + 1,-1));
    return E_OK;
};

err_t i2c_read(void * dev_handler, uint8_t reg_addr, uint8_t *data, uint8_t length)
{
    uint8_t reg_and_data[length + 1];
    reg_and_data[0] = reg_addr;
    data = &reg_and_data; 
    i2c_master_dev_handle_t dev = *(i2c_master_dev_handle_t *)dev_handler;
    ESP_ERROR_CHECK(i2c_master_receive(dev, data, length+1, -1));
    return E_OK;
    
};

err_t i2c_del_master_device(void * dev_handler)
{
    i2c_master_dev_handle_t dev = *(i2c_master_dev_handle_t *)dev_handler;
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev));
    return E_OK;
};

err_t i2c_deinit(void * bus_handler)
{
    i2c_master_bus_handle_t * bus = (i2c_master_bus_handle_t *) bus_handler;
    ESP_ERROR_CHECK(i2c_del_master_bus(bus));
    return E_OK;
};