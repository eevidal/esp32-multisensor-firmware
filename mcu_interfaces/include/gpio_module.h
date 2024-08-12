#ifndef _GPIO_MODULE_H
#define _GPIO_MODULE_H
#include "stdint.h"

#include "error_module.h"

typedef enum {
    GPIO_INPUT = 1,
    GPIO_OUTPUT = 2
} direction_t;

err_t gpio_init(uint8_t gpio_pin, direction_t mode);
err_t gpio_set(uint8_t gpio_pin, direction_t mode);
err_t gpio_send(uint8_t gpio_pin);
err_t gpio_stop(uint8_t gpio_pin);
int gpio_read(uint8_t gpio_pin);

#endif