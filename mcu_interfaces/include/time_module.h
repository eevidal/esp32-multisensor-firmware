#ifndef _TIME_MODULE_H
#define _TIME_MODULE_H
#include <stdint.h>
#include "error_module.h"


/**
 * Delays execution for a specified number of microseconds.
 *
 * @param time The delay duration in microseconds.
 */
void delay_us(int time);

/**
 * Gets the current time in microseconds.
 * 
 * @return The current time in microseconds.
 */
uint64_t  now(void);

/**
 * Calculates the elapsed time in microseconds since a given start time.
 * This function subtracts the start time from the current time to determine the elapsed time in microseconds.
 *
 * @param start_time The start time in microseconds.
 * @return The elapsed time in microseconds.
 */
uint64_t elapsed_time(uint64_t time);

#endif