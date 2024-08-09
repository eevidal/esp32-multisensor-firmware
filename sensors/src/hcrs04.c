
#include <stdio.h>
#include <stdint.h>

#include "error_module.h"
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
} _hcrs04_dev_t;


hcrs04_t* hcrs04_create(uint8_t trigger_pin, uint8_t echo_pin, int timeout){
    _hcrs04_dev_t *sensor = malloc(sizeof(_hcrs04_dev_t));
    sensor->echo_pin = echo_pin;
    sensor->trigger_pin = trigger_pin; 
    sensor->timeout = timeout > 0 ? timeout : TIMEOUT;
 
    CHECK(gpio_set_direction(sensor->echo_pin, GPIO_INPUT));
    CHECK(gpio_set_direction(sensor->trigger_pin, GPIO_OUTPUT));  
    return (hcrs04_t *)sensor;
}; 

err_t hcrs04_send_pulse_and_wait(hcrs04_t *sensor, uint32_t *elapsed){
    _hcrs04_dev_t *sens = (_hcrs04_dev_t *) sensor;
    printf("envio trigger\n");
    CHECK(gpio_send((uint8_t)sens->trigger_pin));
    delay(10);
    CHECK(gpio_stop((uint8_t)sens->trigger_pin));
  
    printf("pido el tiempo\n");
    int init_time = now();
    while(!gpio_read(sens->echo_pin) && (elapsed_time(init_time) <= sens->timeout)); // wait for the echo pin HIGH or timeout
    init_time = now();
    while(gpio_read(sens->echo_pin) && (elapsed_time(init_time) <= sens->timeout)); // wait for the echo pin LOW or timeout
    elapsed = elapsed_time(init_time);
    return OK;

};

float hcrs04_get_distance_m(hcrs04_t *sensor){ 
    printf("a enviar y esperar\n");
    uint32_t elapsed;
    CHECK(hcrs04_send_pulse_and_wait(sensor, &elapsed));
    printf("ya leí\n");
    return ( elapsed / DSOUND_SPEED) * 1000;  // return in meters
};

err_t hcrs04_get_time(hcrs04_t *sensor, int *pulse_width){
    return hcrs04_send_pulse_and_wait(sensor, &pulse_width);
};

err_t hcrs04_delete(hcrs04_t *sensor){
    _hcrs04_dev_t *sens = (_hcrs04_dev_t *) sensor;
    free(sens);
    *sensor = NULL;
    return OK;
};


