#include "time_module.h"
#include "esp_idf.h"
#include "rom/ets_sys.h"

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_timer.h"


#ifndef TARGET_IS_ESP32
#define TARGET_IS_ESP32 1
#endif

#ifndef CONFIG_FREERTOS_HZ
#define CONFIG_FREERTOS_HZ 100  
#endif

#if TARGET_IS_ESP32
void delay(int time){
    ets_delay_us(time*1000);
};

uint64_t now(void){
   // return esp_timer_get_time();
    return esp_timer_get_time();
   
};

uint64_t elapsed_time(uint64_t start_time){
   return ( esp_timer_get_time() - start_time);

};
#endif




