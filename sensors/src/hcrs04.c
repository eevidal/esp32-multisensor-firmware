
#include <stdio.h>
#include <stdint.h>
#include "gpio_module.h"
#include "time_module.h"
#include "hcrs04.h"


#define TIMEOUT   1200 // ms
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
    int elapsed;  /**< Measured pulse width (time of flight) in microseconds. */
} _hcrs04_dev_t;


hcrs04_t* hcrs04_create(int trigger_pin, int echo_pin, int timeout){
    _hcrs04_dev_t *sensor = malloc(sizeof(_hcrs04_dev_t));
    sensor->echo_pin = echo_pin;
    sensor->trigger_pin = trigger_pin; 
    sensor->timeout = timeout > 0 ? timeout : TIMEOUT;
    sensor->elapsed = 0;
    gpio_init(sensor->trigger_pin, GPIO_INPUT);
    gpio_init(sensor->echo_pin, GPIO_OUTPUT);
    return (hcrs04_t *) sensor;
}; 

err_t hcrs04_send_pulse_and_wait(hcrs04_t *sensor){
    _hcrs04_dev_t *sens = (_hcrs04_dev_t *) sensor;
    gpio_send(sens->trigger_pin);
    delay(10);
    gpio_stop(sens->trigger_pin);
  
    
    int init_time = now();
    while(!gpio_read(sens->echo_pin) && (elapsed_time(init_time) <= sens->timeout)); // wait for the echo pin HIGH or timeout
    init_time = now();
    while(gpio_read(sens->echo_pin) && (elapsed_time(init_time) <= sens->timeout)); // wait for the echo pin LOW or timeout
    sens->elapsed = elapsed_time(init_time);
    return OK;

};

float hcrs04_get_distance_m(hcrs04_t *sensor){ 
    hcrs04_send_pulse_and_wait(sensor);
    _hcrs04_dev_t *sens = (_hcrs04_dev_t *) sensor;
    return (sens-> elapsed / DSOUND_SPEED) * 1000;  // return in meters
}; 

err_t hcrs04_get_time(hcrs04_t *sensor, int *pulse_width){
    hcrs04_send_pulse_and_wait(sensor);
    _hcrs04_dev_t *sens = (_hcrs04_dev_t *) sensor;
    (*pulse_width) = sens->elapsed ; 
    return OK;
}
err_t hcrs04_delete(hcrs04_t *sensor){
    _hcrs04_dev_t *sens = (_hcrs04_dev_t *) sensor;
    free(sens);
    *sensor = NULL;
    return OK;
};


