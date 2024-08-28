#ifndef _GPIO_MODULE_H
#define _GPIO_MODULE_H
#include "stdint.h"

#include "error_module.h"

typedef enum {
    GPIO_INPUT = 1,
    GPIO_OUTPUT = 2
} direction_t;

/**
 * Initializes a GPIO pin.
 *
 * This function configures a GPIO pin for input or output, enabling or disabling
 * pull-up and pull-down resistors.
 *
 * @param pin The GPIO pin number.
 * @param mode The GPIO mode:
 *   - GPIO_INPUT: Input mode.
 *   - GPIO_OUTPUT: Output mode.
 * @return E_OK on success.
 */
err_t gpio_init(uint8_t gpio_pin, direction_t mode);

/**
 * Sets the direction of a GPIO pin.
 *
 * This function configures a GPIO pin for input or output.
 *
 * @param pin The GPIO pin number.
 * @param mode The GPIO mode:
 *   - GPIO_INPUT: Input mode.
 *   - GPIO_OUTPUT: Output mode.
 * @return E_OK on success.
 */
err_t gpio_set(uint8_t gpio_pin, direction_t mode);

/**
 * Sets a GPIO pin to HIGH.
 *
 * This function sets the specified GPIO pin to a high level.
 *
 * @param gpio_pin The GPIO pin number.
 * @return E_OK on success.
 */
err_t gpio_send(uint8_t gpio_pin);

/**
 * Sets a GPIO pin to LOW.
 *
 * This function sets the specified GPIO pin to a low level.
 *
 * @param gpio_pin The GPIO pin number.
 * @return E_OK on success.
 */
err_t gpio_stop(uint8_t gpio_pin);

/**
 * Reads the value of a GPIO pin.
 *
 * This function returns the current level of the specified GPIO pin.
 *
 * @param gpio_pin The GPIO pin number.
 * @return The current level of the GPIO pin (0 for LOW, 1 for HIGH).
 */
int gpio_read(uint8_t gpio_pin);

#endif