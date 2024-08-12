
#include <stdio.h>
#include <stdint.h>



#include "hcrs04.h"
#include "mutex_module.h"
#include "error_module.h"
#include "gpio_module.h"
#include "time_module.h"

#define TIMEOUT   6000 // ms
#define DSOUND_SPEED 2.91*2   // mm / micros


/**
 * @brief Structure to hold HC-SR04 sensor data.
 *
 * This structure encapsulates the necessary information for operating an HC-SR04
 * ultrasonic sensor, including GPIO pin configurations, timeout settings, and
 * measured pulse width.
 */
typedef struct{
    int echo_pin;     /**< GPIO pin number for the echo input. */
    int trigger_pin;  /**< GPIO pin number for the trigger output. */
    int timeout; /**< Timeout value in microseconds for the sensor measurement. */
    mutex_t mutex;
} _hcrs04_dev_t;


hcrs04_t* hcrs04_create(uint8_t trigger_pin, uint8_t echo_pin, int timeout){
    _hcrs04_dev_t *sensor = malloc(sizeof(_hcrs04_dev_t));
    sensor->echo_pin = echo_pin;
    sensor->trigger_pin = trigger_pin; 
    sensor->timeout = timeout > 0 ? timeout : TIMEOUT;
    sensor->mutex = mutex_init();
 
    gpio_set(sensor->echo_pin, GPIO_INPUT);
    gpio_set(sensor->trigger_pin, GPIO_OUTPUT);  
    return (hcrs04_t *)sensor;
}; 

err_t hcrs04_send_pulse_and_wait(hcrs04_t *sensor, uint64_t *elapsed){
    _hcrs04_dev_t *sens = (_hcrs04_dev_t *) sensor;
    
    
   
    if (sens->mutex == NULL) 
        return E_FAIL;

   
   // mutex_lock(sens->mutex ); //enter critical region    printf("envio trigger\n");
     
        gpio_stop((uint8_t)sens->trigger_pin);
        delay(4);
        gpio_send((uint8_t)sens->trigger_pin);
        delay(10);
        gpio_stop((uint8_t)sens->trigger_pin);
        uint64_t init_time  = now();       
        while(!gpio_read(sens->echo_pin)&& (elapsed_time(init_time) <= sens->timeout)) ; // wait for the echo pin HIGH or timeout
        uint64_t start_echo  = now();
        uint64_t stop_time ;
        printf("init time %lld\n", init_time);
        while(gpio_read(sens->echo_pin) && (elapsed_time(init_time) <= sens->timeout)) // wait for the echo pin LOW or timeout
            stop_time = now();
         printf("stop time %lld\n", stop_time);

  //  mutex_unlock(sens->mutex ); 
    printf("elapsed %0.04f\n",(float)elapsed_time(start_echo) );   
    *elapsed =  stop_time - start_echo;
   
    return E_OK;

};


err_t hcrs04_get_distance_m(hcrs04_t *sensor,  uint64_t* distance){ 

    uint64_t elapsed;
    hcrs04_send_pulse_and_wait(sensor, &elapsed);
   
    *distance = ( elapsed / DSOUND_SPEED) ;  // return in cm
    return E_OK;
};

err_t hcrs04_get_time(hcrs04_t *sensor, uint64_t *pulse_width){
   hcrs04_send_pulse_and_wait(sensor, pulse_width);
   return E_OK;
};

err_t hcrs04_delete(hcrs04_t *sensor){
    _hcrs04_dev_t *sens = (_hcrs04_dev_t *) sensor;
    free(sens);
    *sensor = NULL;
    return E_OK;
};


