#include "gpio_module.h"
#include "driver/gpio.h"
#include "esp_err.h"

#include <stdint.h>


#define LOW 0
#define HIGH 1

/**
 * @brief 
 * 
 * @param pin 
 * @param mode 
 * @return err_t 
 * 
 */

err_t gpio_init(uint8_t pin, direction_t mode){
    gpio_config_t gpio_conf;
    gpio_conf.pin_bit_mask = (1ULL << pin);  /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
    gpio_conf.mode = (gpio_mode_t)mode;
    gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;    
    printf("gpio\n");
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));
    return OK;
};

err_t gpio_send(uint8_t gpio_pin){
    esp_err_t res;
    printf("seteo nivel \n");
    res = gpio_set_level((gpio_num_t)gpio_pin, (uint32_t)HIGH);
    printf("res %d", (int)res);
    if (res == ESP_OK) 
        return OK;
    else {
        printf("res %d", (int)res);
        return FAIL;
    }
};

err_t gpio_stop(uint8_t gpio_pin){
    return (err_t)gpio_set_level((gpio_num_t)gpio_pin, LOW); 
};

int gpio_read(uint8_t gpio_pin){
    return gpio_get_level((gpio_num_t)gpio_pin);
}