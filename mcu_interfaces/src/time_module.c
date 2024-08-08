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

uint32_t now(void){
    return  xTaskGetTickCount();

};

unsigned long elapsed_time(uint32_t star_time){
    uint32_t _now = xTaskGetTickCount();
    return (star_time - _now)/portTICK_PERIOD_MS;

};
