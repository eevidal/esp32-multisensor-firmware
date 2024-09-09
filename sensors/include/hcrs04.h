#ifndef _HCRS04_H_
#define _HCRS04_H_

#include "error_module.h"
#include "gpio_module.h"
#include "time_module.h"
#include <stdint.h>

typedef void * hcrs04_t;

/**
 * @brief Creates a new HC-SR04 sensor instance.
 *
 * This function allocates memory for a new HC-SR04 sensor structure, initializes its
 * parameters, and configures the GPIO pins for trigger and echo.
 *
 * @param trigger_pin GPIO pin number for the trigger output.
 * @param echo_pin GPIO pin number for the echo input.
 * @param timeout Timeout value in microseconds for the sensor measurement.
 * @return Pointer to the newly created HC-SR04 sensor instance, or NULL on failure.
 */
hcrs04_t* hcrs04_create(uint8_t  trigger_pin, uint8_t echo_pin, int timeout);

/**
 * @brief Gets the distance from the HC-SR04 sensor in meters.
 *
 * This function triggers the HC-SR04 sensor to measure the distance, calculates the
 * distance in meters based on the measured pulse width and the speed of sound,
 * and stores the result in the provided `distance` variable.
 *
 * @param sensor Pointer to the HC-SR04 sensor structure.
 * @param distance Pointer to a float variable where the distance in meters will be stored.
 * @return 
 *     - OK Success
 *     - FAIL Fail
 */
err_t  hcrs04_get_distance_m(hcrs04_t *sensor, float *distance);

/**
 * @brief Gets the pulse width (time of flight) from the HC-SR04 sensor.
 *
 * This function triggers the HC-SR04 sensor to measure the distance, retrieves the
 * measured pulse width, and stores it in the provided `pulse_width` variable.
 *
 * @param sensor Pointer to the HC-SR04 sensor structure.
 * @param pulse_width Pointer to a float variable where the pulse width will be stored.
 * @return 
 *     - OK Success
 *     - FAIL Fail
 */
err_t hcrs04_get_time(hcrs04_t *sensor, uint32_t *pulse_width); 

/**
 * @brief Delete and release the sensor handle
 * 
 * @param sensor  Pointer to the HC-SR04 sensor structure.
 * @return
 *     - OK Success
 *     - FAIL Fail
 */
err_t hcrs04_delete(hcrs04_t *sensor);
int hcrs04_echo_pin(hcrs04_t *sensor);
#endif