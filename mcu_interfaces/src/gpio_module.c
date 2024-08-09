#include "gpio_module.h"
#include "driver/gpio.h"

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
    gpio_conf.mode = mode;
    gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;    
    return (err_t)gpio_config(&gpio_conf);
};

err_t gpio_send(uint8_t gpio_pin){
    return (err_t)gpio_set_level((gpio_num_t)gpio_pin, HIGH);
};

err_t gpio_stop(uint8_t gpio_pin){
    return (err_t)gpio_set_level((gpio_num_t)gpio_pin, LOW); 
};

int gpio_read(uint8_t gpio_pin){
    return gpio_get_level((gpio_num_t)gpio_pin);
}