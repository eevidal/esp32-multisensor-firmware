#include "freertos/timers.h"
#include "logs.h"

unsigned long int time;

err_t delay(time){
    vTaskDelay(10 / port_TICK_MS);
    return OK;
};

err_t now(void);

err_t elapsed_time(unsigned long time);
