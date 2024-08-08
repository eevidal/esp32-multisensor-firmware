#include <stdio.h>

#include "apds9960.h"

#define APDS9960_I2C_ADDRESS    (0x39) 

#define  ID             0x92


typedef struct{
    i2c_bus_t *i2c_dev;
    uint8_t dev_addr;
    uint32_t timeout;
    // apds9960_control_t _control_t; /*< config control register>*/
    // apds9960_enable_t _enable_t;   /*< config enable register>*/
    // apds9960_config1_t _config1_t; /*< config config1 register>*/
    // apds9960_config2_t _config2_t; /*< config config2 register>*/
    // apds9960_config3_t _config3_t; /*< config config3 register>*/
    // apds9960_gconf1_t _gconf1_t;   /*< config gconfig1 register>*/
    // apds9960_gconf2_t _gconf2_t;   /*< config gconfig2 register>*/
    // apds9960_gconf3_t _gconf3_t;   /*< config gconfig3 register>*/
    // apds9960_gconf4_t _gconf4_t;   /*< config gconfig4 register>*/
    // apds9960_status_t _status_t;   /*< config status register>*/
    // apds9960_gstatus_t _gstatus_t; /*< config gstatus register>*/
    // apds9960_propulse_t _ppulse_t; /*< config pro pulse register>*/
    // apds9960_gespulse_t _gpulse_t; /*< config ges pulse register>*/
    // apds9960_pers_t _pers_t;       /*< config pers register>*/
    // uint8_t gest_cnt;              /*< counter of gesture >*/
    // uint8_t up_cnt;                /*< counter of up gesture >*/
    // uint8_t down_cnt;              /*< counter of down gesture >*/
    // uint8_t left_cnt;              /*< counter of left gesture >*/
    // uint8_t right_cnt;             /*< counter of right gesture >*/

}apds9960_dev_t;

apds9960_t *sensor apds9960_init(i2c_bus_t *i2c_params){


};

//i2c_bus_t

apds9960_t apds9960_create(i2c_bus_t *i2c_params)
{
    apds9960_dev_t *sens = (apds9960_dev_t *) malloc(sizeof(apds9960_dev_t));
    if ((void *)sens == NULL) 
        return NULL;
    
    sens->i2c_dev = (i2c_bus_t *)i2c_init_master_mode(i2c_params);
    if (sens->i2c_dev == NULL) {
        free(sens);
        return NULL;
    }
    sens->dev_addr = APDS9960_I2C_ADDRESS; 
    sens->timeout = APDS9960_TIMEOUT_MS_DEFAULT;



    return (apds9960_handle_t) sens;
}
err_t apds9960_delete(apds9960_t *sensor); // RNF.4
err_t apds9960_setup(apds9960_t *sensor, *sensor_params);


err_t apds9960_gesture_init(apds9960_t *sensor)
{
    /* Set default values for ambient light and proximity registers */
    apds9960_set_adc_integration_time(sensor, 10);
    apds9960_set_ambient_light_gain(sensor, APDS9960_AGAIN_4X);
    apds9960_enable_gesture_engine(sensor, false);
    apds9960_enable_proximity_engine(sensor, false);
    apds9960_enable_color_engine(sensor, false);
    apds9960_enable_color_interrupt(sensor, false);
    apds9960_enable_proximity_interrupt(sensor, false);
    apds9960_clear_interrupt(sensor);
    apds9960_enable(sensor, false);
    apds9960_enable(sensor, true);
    apds9960_set_gesture_dimensions(sensor, APDS9960_DIMENSIONS_ALL);
    apds9960_set_gesture_fifo_threshold(sensor, APDS9960_GFIFO_4);
    apds9960_set_gesture_gain(sensor, APDS9960_GGAIN_4X);
    apds9960_set_gesture_proximity_threshold(sensor, 50, 0);
    apds9960_reset_counts(sensor);
    apds9960_set_led_drive_boost(sensor, APDS9960_LEDDRIVE_100MA, APDS9960_LEDBOOST_100PCNT);
    apds9960_set_gesture_waittime(sensor, APDS9960_GWTIME_2_8MS);
    apds9960_set_gesture_pulse(sensor, APDS9960_GPULSELEN_32US, 8);
    apds9960_enable_proximity_engine(sensor, true);
    return apds9960_enable_gesture_engine(sensor, true);
}


err_t apds9960_set_mode(apds9960_t *sensor, apds9960_mode_t mode){
    apds9960_dev_t *sens = (apds9960_dev_t *) sensor;
    uint8_t tmp;

    if (i2c_read_byte(sens->i2c_dev, MODE_ENABLE, &tmp) != OK) {
        return FAIL;
    }

    tmp |= mode;
    return i2c_write_byte(sens->i2c_dev, MODE_ENABLE, tmp);
}

err_t apds9960_proximity_get_mode(apds9960_t *sensor, uint8_t* mode);



err_t apds9960_set_wait_time(apds9960_t sensor, uint8_t time)
{
    apds9960_dev_t *sens = (apds9960_dev_t *) sensor;
    uint8_t val;

    /* Read value from GCONF2 register */
    if (i2c_bus_read_byte(sens->i2c_dev, APDS9960_GCONF2, &val) != ESP_OK) {
        return ESP_FAIL;
    }

    /* Set bits in register to gien value */
    time &= 0x07;
    val &= 0xf8;
    val |= time;
    /* Write register value back into GCONF2 register */
    return i2c_bus_write_byte(sens->i2c_dev, APDS9960_GCONF2, val);
}

}
err_t apds9960_set_threshold(apds9960_t *sensor, uint8_t gesture, uint8_t threshold);
err_t apds9960_set_again(apds9960_t *sensor, uint8_t again);
err_t apds9960_set_ggain(apds9960_t *sensor, uint8_t ggain);
err_t apds9960_set_pgain(apds9960_t *sensor, uint8_t pgain);
err_t apds9960_get_again(apds9960_t *sensor, uint8_t *again);
err_t apds9960_get_ggain(apds9960_t *sensor, uint8_t *ggain);
err_t apds9960_get_pgain(apds9960_t *sensor, uint8_t *pgain);
err_t apds9960_proximity_set_threshold(apds9960_t *sensor,uint8_t threshold);
err_t apds9960_proximity_set_wait(apds9960_t *sensor, uint8_t time);

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