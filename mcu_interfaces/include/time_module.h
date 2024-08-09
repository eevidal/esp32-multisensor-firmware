#ifndef _TIME_MODULE_H
#define _TIME_MODULE_H
#include <stdint.h>
#include "error_module.h"



void delay(unsigned long );

int  now(void);

int elapsed_time(int time);

#endif