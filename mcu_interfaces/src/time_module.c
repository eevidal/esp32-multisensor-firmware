#include "time_module.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "driver/gptimer.h"

#ifndef CONFIG_FREERTOS_HZ
#define CONFIG_FREERTOS_HZ 100  
#endif

void delay(unsigned long time){
    vTaskDelay(pdMS_TO_TICKS(time));
};

int now(void){
    return  (int)xTaskGetTickCount();

};

int elapsed_time(int star_time){
    int _now = (int)xTaskGetTickCount();
    return (int)(star_time - _now)/portTICK_PERIOD_MS;

};
