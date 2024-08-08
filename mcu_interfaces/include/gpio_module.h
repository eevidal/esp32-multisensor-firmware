#ifndef _GPIO_MODULE_H
#define _GPIO_MODULE_H


#include "error_module.h"


typedef enum{

};

typedef enum {
    GPIO_INPUT = 0,
    GPIO_OUTPUT = 1
} direction_t;

err_t gpio_init(int gpio_pin, direction_t mode);
err_t gpio_send(int gpio_pin);
err_t gpio_stop(int gpio_pin);
err_t gpio_read(int gpio_pin);


#endif