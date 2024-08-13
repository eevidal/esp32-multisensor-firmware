#include <esp_idf_version.h>
#include "esp_idf.h"

#include <freertos/FreeRTOS.h>
//#include <freertos/FreeRTOS-Kernel>
#include <freertos/task.h>
#include "freertos/timers.h"
//#include <esp_timer.h>

#include "mutex_module.h"
//#if TARGET_IS_ESP32

//#include <esp_timer.h>


typedef SemaphoreHandle_t sem_t;


mutex_t mutex_init() {
    sem_t * sem = malloc(sizeof(sem_t));
    *sem= xSemaphoreCreateBinary();
    if (sem == NULL) {
        return NULL; // Or handle error differently
    }
    return (mutex_t*)sem; 
    
}


void mutex_lock(mutex_t* mutex){

  xSemaphoreTake(*(sem_t*)mutex, portMAX_DELAY);
};
void mutex_unlock(mutex_t* mutex){
    xSemaphoreGive(*(sem_t*)mutex); 
}


//#endif