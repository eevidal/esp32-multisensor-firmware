#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "i2c_module.h"
#include "logs.h"





typedef struct{


} i2c_dev_t;



i2c_bus_t * i2c_init(w_i2c_config_t *params){
    i2c_config_t conf ; 
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = params->sda_num;
    conf.scl_io_num = params->scl_num;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = params->clk_speed;
    conf.clk_flags = 0;
    
    i2c_param_config(I2C_NUM_0, &conf);
    return (i2c_bus_t *)i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, ESP_INTR_FLAG_LEVEL1);
}



err_t i2c_read_byte(uint8_t dev_addr, uint8_t *data, uint16_t timeout){
    return i2c_master_read_from_device(I2C_NUM_0, dev_addr, &data , 1, pdMS_TO_TICKS(timeout));
}

err_t i2c_read(uint8_t dev_addr, uint8_t length, uint8_t *data, uint16_t timeout){
     return i2c_master_read_from_device(I2C_NUM_0, dev_addr, &data , length, pdMS_TO_TICKS(timeout));
}


err_t i2c_write_byte(uint8_t dev_addr, uint8_t *data){
    return i2c_master_write_to_device(I2C_NUM_0, dev_addr, data, 1, pdMS_TO_TICKS(timeout));
}

err_t i2c_write(uint8_t dev_addr, uint8_t length, uint8_t *data){
    return i2c_master_write_to_device(I2C_NUM_0, dev_addr, data, length, pdMS_TO_TICKS(timeout));
}