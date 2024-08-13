#include "esp_idf.h"

#include "gpio_module.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include <stdint.h>
#include "esp_log.h"

#define LOW 0
#define HIGH 1


    err_t gpio_init(uint8_t pin, direction_t mode)
    {
        gpio_config_t gpio_conf;
        gpio_conf.pin_bit_mask = (1ULL << pin); /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
        gpio_conf.mode = (gpio_mode_t)mode;
        gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&gpio_conf));
        return E_OK;
    };

    err_t gpio_set(uint8_t pin, direction_t mode){
        ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)pin, (gpio_mode_t)mode));
        return E_OK;
    }

    err_t gpio_send(uint8_t gpio_pin)
    {   
 

        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)gpio_pin, (uint32_t)HIGH));


        return E_OK;
    };

    err_t gpio_stop(uint8_t gpio_pin)
    {
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)gpio_pin, (uint32_t)LOW));
        return E_OK;
    };

    int gpio_read(uint8_t gpio_pin)
    {
           return gpio_get_level((gpio_num_t)gpio_pin);
    };

