#ifndef _APDS9960_H_
#define _APDS9960_H_

#include "logs.h"
#include <stdint.h>
#include <stdbool.h>
#include "i2c_module.h"

typedef void *apds9960_t;

typedef struct{

} i2c_bus_t;

/* Acceptable parameters for setMode */
typedef enum {
    APDS9960_POWER              = 0,
    APDS9960_AMBIENT_LIGHT      = 1,
    APDS9960_PROXIMITY          = 2,
    APDS9960_WAIT               = 3,
    APDS9960_AMBIENT_LIGHT_INT  = 4,
    APDS9960_PROXIMITY_INT      = 5,
    APDS9960_GESTURE            = 6,
    APDS9960_ALL                = 7,
} apds9960_mode_t;

/* Gesture wait time values */
typedef enum {
    APDS9960_GWTIME_0MS     = 0b00000000,  
    APDS9960_GWTIME_2_8MS   = 0b00000001,
    APDS9960_GWTIME_5_6MS   = 0b00000010,
    APDS9960_GWTIME_8_4MS   = 0b00000011,
    APDS9960_GWTIME_14_0MS  = 0b00000100,
    APDS9960_GWTIME_22_4MS  = 0b00000101,
    APDS9960_GWTIME_30_8MS  = 0b00000110,
    APDS9960_GWTIME_39_2MS  = 0b00000111
} apds9960_gwtime_t;


typedef enum{
    NONE,
    LEFT,
    RIGHT,
    UP,
    DOWN,
    NEAR,
    FAR,
    ALL
} apds9960_gesture_t;

apds9960_t *sensor apds9960_init(i2c_bus_t *i2c_params);
err_t apds9960_delete(apds9960_t *sensor); // RNF.4
err_t apds9960_setup(apds9960_t *sensor, *sensor_params);

/**
 * @brief Configure work mode
 *
 * @param sensor object handle of apds9960
 * @param mode one of apds9960_mode_t struct
 *
 * @return
 *     - OK Success
 *     - FAIL on Fail
 */
err_t apds9960_set_mode(apds9960_t *sensor, apds9960_mode_t mode);


/**
 * @brief Get work mode
 *
 * @param sensor object handle of apds9960
 * @param mode one pointer to a apds9960_mode_t struct
 * @return
 *     - OK Success
 *     - FAIL on Fail
 */
err_t apds9960_get_mode(apds9960_t *sensor, apds9960_mode_t* mode);

/**
 * @brief Sets the time in low power mode between gesture detections
 * 
 * @param sensor 
 * @param time 
 * @return err_t 
 */
err_t apds9960_set_wait_time(apds9960_t sensor, uint8_t time);


err_t apds9960_set_threshold(apds9960_t *sensor, uint8_t gesture, uint8_t threshold);
err_t apds9960_set_again(apds9960_t *sensor, uint8_t again);
err_t apds9960_set_ggain(apds9960_t *sensor, uint8_t ggain);
err_t apds9960_set_pgain(apds9960_t *sensor, uint8_t pgain);
err_t apds9960_get_again(apds9960_t *sensor, uint8_t *again);
err_t apds9960_get_ggain(apds9960_t *sensor, uint8_t *ggain);
err_t apds9960_get_pgain(apds9960_t *sensor, uint8_t *pgain);
err_t apds9960_proximity_set_threshold(apds9960_t *sensor,uint8_t threshold);
err_t apds9960_proximity_set_wait(apds9960_t *sensor, uint8_t time);
err_t apds9960_gesture_init(apds9960_t *sensor);
err_t apds9960_read_raw_data(uint8_t *gesture_data);
err_t apds9960_read_gesture(apds9960_t *sensor, gesture_t *gesture);
err_t apds9960_set_gesture_offset(apds9960_t *sensor,
uint8_t offset_up, uint8_t offset_down, uint8_t offset_left, uint8_t offset_right);
err_t apds9960_proximity_init(apds9960_t *sensor);
err_t apds9960_read_raw_proximity(apds9960_t *sensor, uint8_t *proximity);
err_t apds9960_read_proximity(apds9960_t *sensor, uint8_t *proximity);
err_t apds9960_get_color_data(apds9960_t sensor, uint16_t *r,
uint16_t *g, uint16_t *b, uint16_t *c);
err_t apds9960_enable_color_engine(apds9960_t sensor, bool en);
err_t apds9960_get_ambient_light(apds9960_t *sensor, uint16_t *l);

// Private Functions 
err_t apds9960_read_byte(apds9960_t sensor, uint8_t addr,uint8_t *data);
err_t apds9960_read(apds9960_t sensor, uint8_t addr, uint8_t *buf, uint8_t len);
err_t apds9960_write_byte(apds9960_t sensor, uint8_t addr,uint8_t data);
err_t apds9960_write(apds9960_t sensor, uint8_t addr, uint8_t *buf, uint8_t len);
err_t process_gesture(apds9960_t sensor, uint8_t data,gesture_t *gesture);
err_t decode_gesture(apds9960_t sensor, uint8_t data,gesture_t *gesture);
err_t apds9960_calculate_lux(apds9960_t sensor, uint16_t r,
uint16_t g, uint16_t b, uint16_t *l);

#endif