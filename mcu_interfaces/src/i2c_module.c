
#include "freertos/FreeRTOS.h"

#include "esp_err.h"
#include "i2c_module.h"
#include "error_module.h"
#include "esp_log.h"
#include <string.h>
#define I2C_MASTER_FREQ_HZ 100000

#ifndef CONFIG_FREERTOS_HZ
#define CONFIG_FREERTOS_HZ 100
#endif

#define I2C_NUM I2C_NUM_0
#define NULL_I2C_MEM_ADDR 0xFF
#define I2C_ACK_CHECK_EN 0x1

static const char *tag = "I2C Module:";

#define  _LEGACY
#ifdef _LEGACY

#include "driver/i2c.h"

typedef struct i2c_master_t{
    int port;
}i2c_master_t; 

typedef struct i2c_master_dev_handle_t{
    int port;
    uint8_t dev_addr;
} i2c_master_dev_handle_t;

i2c_bus_t *i2c_init_bus(const w_i2c_config_t *params)
{
    int i2c_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.master.clk_speed = params->clk_speed;
    conf.sda_io_num = params->sda_num;
    conf.scl_io_num = params->scl_num;
    conf.sda_pullup_en = true;
    conf.scl_pullup_en = true;
    conf.clk_flags = 0;
    ESP_ERROR_CHECK(i2c_param_config(i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port, conf.mode, 0, 0, 0));
    i2c_master_t *bus_handle = malloc(sizeof(i2c_master_t));
    bus_handle->port = i2c_port; 
   // ESP_ERROR_CHECK(i2c_new_master_bus(&conf, &bus_handle));
    return (i2c_bus_t *)bus_handle;
};

err_t i2c_deinit(void *bus_handler)
{
    i2c_master_t *bus = (i2c_master_t *)bus_handler;
    free(bus);
    return E_OK;
};

i2c_dev_t *i2c_add_master_device(uint16_t dev_addr, uint32_t cl_speed, i2c_bus_t *bus_handle){
    i2c_master_dev_handle_t *handle = malloc(sizeof(i2c_master_dev_handle_t));
   // i2c_master_t * bus = (i2c_master_t *)bus_handle;
    handle->port =((i2c_master_t*)bus_handle)->port;    
    handle->dev_addr = dev_addr << 1;
    return (i2c_dev_t *)handle;
}

/*err_t i2c_write_to_device(i2c_port_t i2c_num, uint8_t device_address,
                                     const uint8_t* write_buffer, size_t write_size,
                                     TickType_t ticks_to_wait)*/

err_t i2c_write(void*sensor, uint8_t reg_addr, uint8_t  data, uint8_t length){
    i2c_master_dev_handle_t *sens = (i2c_master_dev_handle_t*)sensor;
    if (sens == NULL)
        printf("NULL");
    uint8_t addr = sens->dev_addr;
    uint8_t port = sens->port;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_LOGI(tag, "Escribiendo en %X y registro %X\n", addr >> 1, reg_addr);
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, addr | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addr, true));
    ESP_ERROR_CHECK(i2c_master_write(cmd, &data, length, I2C_MASTER_LAST_NACK));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(port, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
    return E_OK;
}

err_t i2c_read(void *dev_handler,  uint8_t reg_addr, uint8_t *data, uint8_t length){
    i2c_master_dev_handle_t* sens = (i2c_master_dev_handle_t*)dev_handler;
    uint8_t addr = sens->dev_addr;
    uint8_t port = sens->port;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, addr | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addr, true));
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, addr | I2C_MASTER_READ, true));
    ESP_ERROR_CHECK(i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(port, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
    return E_OK;

}

#else
#include "driver/i2c_master.h"
i2c_bus_t *i2c_init_bus(const w_i2c_config_t *params)
{
    i2c_master_bus_config_t conf;
    conf.i2c_port = I2C_NUM_0; // autoconfig
    conf.clk_source = SOC_CPU_CLK_SRC_PLL; // I2C_CLK_SRC_DEFAULT;
    conf.sda_io_num = params->sda_num;
    conf.scl_io_num = params->scl_num;
    conf.glitch_ignore_cnt = 7;
    conf.intr_priority = 0;
    conf.trans_queue_depth = 16;
    conf.flags.enable_internal_pullup = false;
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&conf, &bus_handle));
    return (i2c_bus_t *)bus_handle;
};

i2c_dev_t *i2c_add_master_device(uint16_t dev_addr, uint32_t cl_speed, i2c_bus_t *bus_handle)
{
    i2c_device_config_t cfg;
    cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg.device_address = dev_addr << 1;
    cfg.scl_speed_hz = cl_speed;
    i2c_master_dev_handle_t *handle = malloc(sizeof(i2c_master_dev_handle_t));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(((i2c_master_bus_handle_t)bus_handle), &cfg, handle));
    return (i2c_dev_t *)handle;
};

err_t i2c_write(void *dev_handler, uint8_t reg_addr, uint8_t data, uint8_t length)
{
    uint8_t reg_and_data[length + 1];
    reg_and_data[0] = reg_addr;
    reg_and_data[1] = data;
    ESP_LOGI(tag, "Write on device 0x%X -> 0x%X \n", reg_addr, data);
    ESP_ERROR_CHECK(i2c_master_transmit((i2c_master_dev_handle_t)dev_handler, reg_and_data, length + 1, -1));
    return E_OK;
};

err_t i2c_read(void *dev_handler, const uint8_t reg_addr, uint8_t *data, uint8_t length)
{
    uint8_t tx_data[] = {reg_addr};
    uint8_t rx_data[length];
    //nt8_t buffer[length];
    i2c_master_dev_handle_t dev = *(i2c_master_dev_handle_t *)dev_handler;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev, tx_data, sizeof(tx_data), rx_data, length, -1));
   // ESP_ERROR_CHECK(i2c_master_receive(dev,buf, (length+1), -1));
    memcpy(data, rx_data, length);

    
   // ESP_LOGD(tag, "Read from device 0x%X -> 0x%X \n", (int)reg_addr, buffer[0]);
    ESP_LOGD(tag, "Read from device 0x%X -> 0x%X \n", (int)reg_addr, *data);
    return E_OK;
};

err_t i2c_del_master_device(void *dev_handler)
{
    i2c_master_dev_handle_t dev = *(i2c_master_dev_handle_t *)dev_handler;
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev));
    return E_OK;
};

err_t i2c_deinit(void *bus_handler)
{
    i2c_master_bus_handle_t bus = *(i2c_master_bus_handle_t *)bus_handler;
    ESP_ERROR_CHECK(i2c_del_master_bus(bus));
    return E_OK;
};
#endif