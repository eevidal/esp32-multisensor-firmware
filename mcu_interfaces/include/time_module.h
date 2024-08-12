#ifndef _TIME_MODULE_H
#define _TIME_MODULE_H
#include <stdint.h>
#include "error_module.h"


/**
 * @brief 
 * 
 * @param time 
 */
void delay_us(int time);

uint64_t  now(void);

uint64_t elapsed_time(uint64_t time);

#endif