#include <stdio.h>

#include "apds9960.h"
#include "apds9960_internals.h"
#include "error_module.h"

#define APDS9960_I2C_ADDRESS (0x39)
#define MAX_CLK 400000

#define ID 0x92

#define CAST(X) ((apds9960_dev_t *)(*X))

typedef struct
{
    uint8_t dev_addr;
    i2c_dev_t *i2c_dev_hadler;
    uint32_t i2c_clk;
    uint8_t enable;   /** enable register 0x80*/
    uint8_t status;   /**status 0x93*/
    uint8_t gstatus;  /**status 0xAF */
    uint8_t control;  /**Gain Control 0x8F*/
    uint8_t config1;  /**0x8D*/
    uint8_t config2;  /**0x90*/
    uint8_t config3;  /**0x9F*/
    uint8_t gconfig1; /**0xA2*/
    uint8_t gconfig2; /**0xA3*/
    uint8_t gconfig3; /**0xAA*/
    uint8_t gconfig4; /**0xAB*/
    uint8_t pers;     /**0x8C*/
    uint8_t wtime;    /**0x8D*/
    uint8_t atime;    /**0x8C*/
    uint8_t ppulse;   /**0x8E*/
    uint8_t gpulse;   /**0xA6*/
    uint8_t gmode;     /**0xAB<0>*/

    // apds9960_propulse_t _ppulse_t; /*< config pro pulse register>*/
    // apds9960_gespulse_t _gpulse_t; /*< config ges pulse register>*/

    // uint8_t gest_cnt;              /*< counter of gesture >*/
    // uint8_t up_cnt;                /*< counter of up gesture >*/
    // uint8_t down_cnt;              /*< counter of down gesture >*/
    // uint8_t left_cnt;              /*< counter of left gesture >*/
    // uint8_t right_cnt;             /*< counter of right gesture >*/

} apds9960_dev_t;

apds9960_t *apds9960_init(i2c_bus_t *i2c_bus)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)malloc(sizeof(apds9960_dev_t));
    if ((void *)sens == NULL || (void *)i2c_bus == NULL)
        return NULL;

    sens->dev_addr = APDS9960_I2C_ADDRESS;
    sens->i2c_clk = MAX_CLK;
    i2c_dev_t dev = NULL;
    i2c_add_master_device(1, sens->dev_addr, sens->i2c_clk, i2c_bus, &dev);
    if (dev == NULL)
        return NULL;

    sens->i2c_dev_hadler = dev;
    sens->enable = 0x00;
    sens->status = 0x00;
    sens->gstatus = 0x00;
    sens->config1 = 0x60; // Bit 6 and 5 are reserved, and are automatically set to 1 at POR.
    sens->config2 = 0x01;
    sens->config3 = 0x00;
    sens->gconfig1 = 0x00;
    sens->gconfig2 = 0x00;
    sens->gconfig3 = 0x00;
    sens->gconfig4 = 0x00;
    sens->pers = 0x00;
    sens->wtime = 0xFF;
    sens->atime = 0xFF;
    sens->ppulse = 0x40;
    sens->gmode = 0x00;

    /**.........continuara.....*/
    return (apds9960_t)sens;
}

err_t apds9960_delete(apds9960_t *sensor)
{
    if (*sensor == NULL)
    {
        return E_OK;
    }
    apds9960_dev_t *sens = (apds9960_dev_t *)(*sensor);
    i2c_del_master_device(sens->i2c_dev_hadler);
    free(sens);
    *sensor = NULL;
    return E_OK;
}; // RNF.4

err_t apds9960_write(apds9960_dev_t *sensor, uint8_t addr, uint8_t *buf, uint8_t len)
{
    return (i2c_write(sensor->i2c_dev_hadler, addr, buf, len));
}

err_t apds9960_read(apds9960_dev_t *sensor, uint8_t addr, uint8_t *buf, uint8_t len)
{
    return (i2c_read(sensor->i2c_dev_hadler, addr, &buf, len));
}

err_t apds9960_enable_engine(apds9960_t *sensor, apds9960_mode_t mode)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)(*sensor);
    switch (mode)
    {
    case APDS9960_POWER:
        sens->enable |= PON;
        break;
    case APDS9960_ALS:
        sens->enable |= AEN; // check control first;
        break;
    case APDS9960_PROXIMIMTY:
        sens->enable |= PEN; // check control first
        break;
    case APDS9960_WAIT:
        sens->enable |= WEN;
        break;
    case APDS9960_AINT:
        sens->enable |= AIEN;
        break;
    case APDS9960_PINT:
        sens->enable |= PIEN;
        break;
    case APDS9960_GESTURE: // check control
        sens->enable |= GEN;
        break;
    case APDS9960_ALL:
        sens->enable |= ALL;
    default:
        break;
    }
    uint8_t data = sens->enable;
    i2c_write(sens->i2c_dev_hadler, ENABLE, &data, 1);
    return E_OK;
}

err_t apds9960_disable_engine(apds9960_t *sensor, apds9960_mode_t mode)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)(*sensor);
    switch (mode)
    {
    case APDS9960_POWER:
        sens->enable &= ~PON;
        break;
    case APDS9960_ALS:
        sens->enable &= ~AEN;
        break;
    case APDS9960_PROXIMIMTY:
        sens->enable &= ~PEN;
        break;
    case APDS9960_WAIT:
        sens->enable &= ~WEN;
        break;
    case APDS9960_AINT:
        sens->enable &= ~AIEN;
        break;
    case APDS9960_PINT:
        sens->enable &= ~PIEN;
        break;
    case APDS9960_GESTURE:
        sens->enable &= ~GEN;
        break;
    case APDS9960_ALL:
        sens->enable &= 0x40;
    default:
        break;
    }
    uint8_t data = sens->enable;
    i2c_write(sens->i2c_dev_hadler, ENABLE, &data, 1);
    return E_OK;
}

err_t apds9960_als_set_threshold(apds9960_t *sensor, uint16_t low, uint16_t high)
{
    apds9960_dev_t *sens = CAST(sensor);
    apds9960_write(sens, AILTL, (low & 0xFF), 1);
    apds9960_write(sens, AILTH, (low >> 8), 1);
    apds9960_write(sens, AIHTL, (high & 0xFF), 1);
    apds9960_write(sens, AIHTH, (high >> 8), 1);
    return E_OK;
}

err_t apds9960_proximity_set_threshold(apds9960_t *sensor, uint8_t threshold, uint8_t persistence)
{
    apds9960_dev_t *sens = CAST(sensor);
    apds9960_write(sens, PILT, (threshold & 0xFF), 1);
    apds9960_write(sens, PIHT, (threshold >> 8), 1);

    if (persistence > 7)
        persistence = 7;

    sens->pers &= 0x0F;
    sens->pers |= (persistence << 4);

    apds9960_write(sens, PERS, persistence, 1);
    return E_OK;
}

err_t apds9960_gesture_set_threshold(apds9960_t *sensor, uint8_t gpenth, uint8_t gexth)
{
    apds9960_dev_t *sens = CAST(sensor);
    apds9960_write(sens, GPENTH, gpenth, 1);
    apds9960_write(sens, GEXTH, gexth, 1);
    return E_OK;
}

// err_t apds9960_setup(apds9960_t *sensor, *sensor_params);

// err_t apds9960_gesture_init(apds9960_t *sensor)
// {
//    or /* Set default values f ambient light and proximity registers */
//     apds9960_set_adc_integration_time(sensor, 10);

//     apds9960_set_ambient_light_gain(sensor, APDS9960_AGAIN_4X);

//     apds9960_enable_gesture_engine(sensor, false);
//     apds9960_enable_proximity_engine(sensor, false);
//     apds9960_enable_color_engine(sensor, false);
//     apds9960_enable_color_interrupt(sensor, false);
//     apds9960_enable_proximity_interrupt(sensor, false);

//     apds9960_clear_interrupt(sensor);
//     apds9960_enable(sensor, false);
//     apds9960_enable(sensor, true);
//     apds9960_set_gesture_dimensions(sensor, APDS9960_DIMENSIONS_ALL);
//     apds9960_set_gesture_fifo_threshold(sensor, APDS9960_GFIFO_4);
//     apds9960_set_gesture_gain(sensor, APDS9960_GGAIN_4X);
//     apds9960_set_gesture_proximity_threshold(sensor, 50, 0);
// apds9960_reset_counts(sensor);
//     apds9960_set_led_drive_boost(sensor, APDS9960_LEDDRIVE_100MA, APDS9960_LEDBOOST_100PCNT);
//     apds9960_set_gesture_waittime(sensor, APDS9960_GWTIME_2_8MS);
//     apds9960_set_gesture_pulse(sensor, APDS9960_GPULSELEN_32US, 8);
//     apds9960_enable_proximity_engine(sensor, true);
//     return apds9960_enable_gesture_engine(sensor, true);
// }

// err_t apds9960_set_mode(apds9960_t *sensor, apds9960_mode_t mode){
//     apds9960_dev_t *sens = (apds9960_dev_t *) sensor;
//     uint8_t tmp;

//     if (i2c_read_byte(sens->i2c_dev, MODE_ENABLE, &tmp) != OK) {
//         return FAIL;
//     }

//     tmp |= mode;
//     return i2c_write_byte(sens->i2c_dev, MODE_ENABLE, tmp);
// }

// err_t apds9960_proximity_get_mode(apds9960_t *sensor, uint8_t* mode);

// err_t apds9960_set_wait_time(apds9960_t sensor, uint8_t time)
// {
//     apds9960_dev_t *sens = (apds9960_dev_t *) sensor;
//     uint8_t val;

//     /* Read value from GCONF2 register */
//     if (i2c_bus_read_byte(sens->i2c_dev, APDS9960_GCONF2, &val) != ESP_OK) {
//         return ESP_FAIL;
//     }

//     /* Set bits in register to gien value */
//     time &= 0x07;
//     val &= 0xf8;
//     val |= time;
//     /* Write register value back into GCONF2 register */
//     return i2c_bus_write_byte(sens->i2c_dev, APDS9960_GCONF2, val);
// }

// }

// CONTROL Register

err_t apds9960_set_again(apds9960_t *sensor, apds9960_again_t again)
{
    apds9960_dev_t *sens = CAST(sensor);
    uint8_t temp = 0x03 & again;
    sens->control |= temp;
    temp = sens->control;
    return (apds9960_write(sens, CONTROL, &temp, 1));
}

err_t apds9960_set_pgain(apds9960_t *sensor, apds9960_gain_t pgain)
{
    apds9960_dev_t *sens = CAST(sensor);
    uint8_t temp = 0x0C & pgain;
    sens->control |= temp;
    temp = sens->control;
    return (apds9960_write(sens, CONTROL, &temp, 1));
};

err_t apds9960_set_ldrive(apds9960_t *sensor, apds9960_ldrive_t ldrive)
{
    apds9960_dev_t *sens = CAST(sensor);
    uint8_t temp = 0xC0 & ldrive;
    sens->control |= temp;
    temp = sens->control;
    return (apds9960_write(sens, CONTROL, &temp, 1));
};

// CONFIG2
err_t apds9960_set_ggain(apds9960_t *sensor, apds9960_gain_t ggain)
{
    apds9960_dev_t *sens = CAST(sensor);
    uint8_t temp = 0x60 & ggain;
    sens->gconfig2 |= temp;
    temp = sens->gconfig2;
    return (apds9960_write(sens, GCONF2, &temp, 1));
};

err_t apds9960_set_gldrive(apds9960_t *sensor, apds9960_ldrive_t gldrive)
{
    apds9960_dev_t *sens = CAST(sensor);
    uint8_t temp = 0x14 & gldrive;
    sens->gconfig2 |= temp;
    temp = sens->gconfig2;
    return (apds9960_write(sens, GCONF2, &temp, 1));
};

err_t apds9960_set_gwtime(apds9960_t *sensor, apds9960_gwtime_t gwtime)
{
    apds9960_dev_t *sens = CAST(sensor);
    uint8_t temp = 0x07 & gwtime;
    sens->gconfig2 |= temp;
    temp = sens->gconfig2;
    return (apds9960_write(sens, GCONF2, &temp, 1));
};

// WTIME
err_t apds9960_set_wtime(apds9960_t *sensor, uint8_t wtime)
{
    apds9960_dev_t *sens = CAST(sensor);
    sens->wtime = wtime;
    return (apds9960_write(sens, WTIME, &wtime, 1));
}

// ATIME
err_t apds9960_set_atime(apds9960_t *sensor, uint8_t atime)
{
    apds9960_dev_t *sens = CAST(sensor);
    sens->atime = atime;
    return (apds9960_write(sens, ATIME, &atime, 1));
}

// WLONG
err_t apds9960_enable_wlong(apds9960_t *sensor)
{
    apds9960_dev_t *sens = CAST(sensor);
    uint8_t config1 = 0x61;
    sens->config1 = config1;
    return (apds9960_write(sens, CONFIG1, &config1, 1));
}
err_t apds9960_disable_wlong(apds9960_t *sensor)
{
    apds9960_dev_t *sens = CAST(sensor);
    uint8_t config1 = 0x60;
    sens->config1 = config1;
    return (apds9960_write(sens, CONFIG1, &config1, 1));
}

// PPULSE
err_t apds9960_set_proximity_pulse(apds9960_t *sensor, apds9960_pulse_len_t len, uint8_t pulses)
{
    apds9960_dev_t *sens = CAST(sensor);
    uint8_t tmp = 0;
    if (pulses < 1)
        pulses = 1;
    if (pulses > 64)
        pulses = 64;
    pulses--;
    sens->ppulse = pulses | len;
    pulses = sens->ppulse;
    return (apds9960_write(sens, GPULSE, &tmp, 1));
}

// CONFIG2
err_t apds9960_set_proximity_sat_int_(apds9960_t *sensor, apds9960_psat_int_t sat)
{
    apds9960_dev_t *sens = CAST(sensor);
    uint8_t config2 = 0x00;
    config2 |= sat;
    sens->config2 = config2;
    return (apds9960_write(sens, CONFIG2, &config2, 1));
}

err_t apds9960_set_proximity_clear_int_(apds9960_t *sensor, apds9960_pclear_int_t clear)
{
    apds9960_dev_t *sens = CAST(sensor);
    uint8_t config2 = 0x00;
    config2 |= clear;
    sens->config2 = config2;
    return (apds9960_write(sens, CONFIG2, &config2, 1));
}

err_t apds9960_set_ledboost(apds9960_t *sensor, apds9960_ledboost_t boost)
{
    apds9960_dev_t *sens = CAST(sensor);
    uint8_t config2 = 0x00;
    config2 |= boost;
    sens->config2 = config2;
    return (apds9960_write(sens, CONFIG2, &config2, 1));
}

// P_OFFSETS
err_t apds9960_set_poffset_ur(apds9960_t *sensor, uint8_t offset)
{
    apds9960_dev_t *sens = CAST(sensor);
    return (apds9960_write(sens, POFFSET_UR, &offset, 1));
}
err_t apds9960_set_poffset_dl(apds9960_t *sensor, uint8_t offset)
{
    apds9960_dev_t *sens = CAST(sensor);
    return (apds9960_write(sens, POFFSET_DL, &offset, 1));
}

//CONFIG3
err_t apds9960_set_proximity_pcmp(apds9960_t *sensor, apds9960_pcmp_t pcmd_val)
{
    apds9960_dev_t *sens = CAST(sensor);
    sens->config3 |= pcmd_val;
    uint8_t config3 = sens->config3;
    return (apds9960_write(sens, CONFIG3, &config3, 1));
}

err_t apds9960_set_proximity_sai(apds9960_t *sensor, apds9960_psai_t sai_val)
{
    apds9960_dev_t *sens = CAST(sensor);
    sens->config3 |= sai_val;
    uint8_t config3 = sens->config3;
    return (apds9960_write(sens, CONFIG3, &config3, 1));
}

err_t apds9960_set_proximity_mask(apds9960_t *sensor, apds9960_pmask_t mask);
{
    apds9960_dev_t *sens = CAST(sensor);
    sens->config3 |= mask;
    uint8_t config3 = sens->config3;
    return (apds9960_write(sens, CONFIG3, &config3, 1));
}
//GCONFIG1

err_t apds9960_set_gestrure_gexmsk(apds9960_t *sensor, apds9960_gexmsk_t mask);
{
    apds9960_dev_t *sens = CAST(sensor);
    sens->gconfig1 |= mask;
    uint8_t gconfig1 = sens->gconfig1;
    return (apds9960_write(sens, GCONFIG1, &gconfig1, 1));
}

err_t apds9960_set_gestrure_fifoth(apds9960_t *sensor, apds9960_gfifoth_t fifoth);
{
    apds9960_dev_t *sens = CAST(sensor);
    sens->gconfig1 |= fifoth;
    uint8_t gconfig1 = sens->gconfig1;
    return (apds9960_write(sens, GCONFIG1, &gconfig1, 1));
}
err_t apds9960_set_gestrure_gexpers(apds9960_t *sensor, apds9960_gexpers_t gexper);
{
    apds9960_dev_t *sens = CAST(sensor);
    sens->gconfig1 |= gexper;
    uint8_t gconfig1 = sens->gconfig1;
    return (apds9960_write(sens, GCONFIG1, &gconfig1, 1));
}

//CONFIG GOFFSET
err_t apds9960_set_gestrure_offsets(apds9960_t *sensor, uint8_t offset_up, uint8_t offset_down, uint8_t offset_left, uint8_t offset_right);
{
    apds9960_dev_t *sens = CAST(sensor);
    apds9960_write(sens, GOFFSET_D, &offset_down,1);
    apds9960_write(sens, GOFFSET_U, &offset_up,1);
    apds9960_write(sens, GOFFSET_L, &offset_left,1);
    apds9960_write(sens, GOFFSET_R, &offset_right,1);
    return E_OK;
}

//GPULSE
err_t apds9960_set_gesture_pulse(apds9960_t *sensor, apds9960_pulse_len_t len, uint8_t pulses)
{
    apds9960_dev_t *sens = CAST(sensor);
    uint8_t tmp = 0;
    if (pulses < 1)
        pulses = 1;
    if (pulses > 64)
        pulses = 64;
    pulses--;
    sens->gpulse = pulses | len;
    pulses = sens->gpulse;
    return (apds9960_write(sens, GPULSE, &tmp, 1));
}

//GCONF3
err_t apds9960_set_gestrure_gdims(apds9960_t *sensor, apds9960_gdims_t gdim);
{
    apds9960_dev_t *sens = CAST(sensor);
    sens->gconfig3 |= gdim;
    uint8_t gconfig3 = sens->gconfig3;
    return (apds9960_write(sens, GCONF3, &gconfig3, 1));
}


err_t apds9960_set_gesture_int_on(apds9960_t *sensor){
    apds9960_dev_t *sens = CAST(sensor);
     sens->gconfig4 |=  0b00000010;
    uint8_t gconfig4 = sens->gconfig4;
    return (apds9960_write(sens, GCONF4, &gconfig4, 1));

}

err_t apds9960_set_gesture_int_off(apds9960_t *sensor){
    apds9960_dev_t *sens = CAST(sensor);
     sens->gconfig4 &=  0b11111101;
    uint8_t gconfig4 = sens->gconfig4;
    return (apds9960_write(sens, GCONF4, &gconfig4, 1));

}

//INTERRUPS
err_t apds9960_set_force_interrupt(apds9960_t *sensor, uint8_t iforce){
     apds9960_dev_t *sens = CAST(sensor);
    return (apds9960_write(sens, IFORCE, &iforce, 1));
}
err_t apds9960_set_proximity_clear_int(apds9960_t *sensor, uint8_t piclear){
     apds9960_dev_t *sens = CAST(sensor);
    return (apds9960_write(sens, PICLEAR, &piclear, 1));
}
err_t apds9960_set_color_clear_int(apds9960_t *sensor, uint8_t ciclear){
     apds9960_dev_t *sens = CAST(sensor);
    return (apds9960_write(sens, CICLEAR, &ciclear, 1));
}
err_t apds9960_set_als_clear_int(apds9960_t *sensor, uint8_t aiclear){
     apds9960_dev_t *sens = CAST(sensor);
    return (apds9960_write(sens, AICLEAR, &aiclear, 1));
}


err_t apds9960_get_again(apds9960_t *sensor, uint8_t *again);
err_t apds9960_get_ggain(apds9960_t *sensor, uint8_t *ggain);
err_t apds9960_get_pgain(apds9960_t *sensor, uint8_t *pgain);

// err_t apds9960_proximity_set_threshold(apds9960_t *sensor,uint8_t threshold);
// err_t apds9960_proximity_set_wait(apds9960_t *sensor, uint8_t time);

// err_t apds9960_read_raw_data(uint8_t *gesture_data);

err_t apds9960_read_gesture(apds9960_t *sensor, apds9960_gesture_t *gesture)
{
    uint8_t toRead;
    uint8_t buf[256];
    unsigned long t = 0;
    uint8_t gesture_r;
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;

    while (1)
    {
        int up_down_diff = 0;
        int left_right_diff = 0;
        gesture_r = 0;

        if (!apds9960_gesture_valid(sensor))
        {
            return 0;
        }

        delay_us(30);
        i2c_bus_read_byte(sens->i2c_dev, GFLVL, &toRead);
        i2c_bus_read_bytes(sens->i2c_dev, GFIFO_U, toRead, buf);
        i2c_read_byte(sens->dev_addr, );

        if (abs((int)buf[0] - (int)buf[1]) > 13)
        {
            up_down_diff += (int)buf[0] - (int)buf[1];
        }

        if (abs((int)buf[2] - (int)buf[3]) > 13)
        {
            left_right_diff += (int)buf[2] - (int)buf[3];
        }

        if (up_down_diff != 0)
        {
            if (up_down_diff < 0)
            {
                if (sens->down_cnt > 0)
                {
                    gestureReceived = UP;
                }
                else
                {
                    sens->up_cnt++;
                }
            }
            else if (up_down_diff > 0)
            {
                if (sens->up_cnt > 0)
                {
                    gestureReceived = DOWN;
                }
                else
                {
                    sens->down_cnt++;
                }
            }
        }

        if (left_right_diff != 0)
        {
            if (left_right_diff < 0)
            {
                if (sens->right_cnt > 0)
                {
                    gestureReceived = LEFT;
                }
                else
                {
                    sens->left_cnt++;
                }
            }
            else if (left_right_diff > 0)
            {
                if (sens->left_cnt > 0)
                {
                    gestureReceived = RIGHT;
                }
                else
                {
                    sens->right_cnt++;
                }
            }
        }

        if (up_down_diff != 0 || left_right_diff != 0)
        {
            t = xTaskGetTickCount();
        }

        if (gestureReceived || xTaskGetTickCount() - t > (300 / portTICK_RATE_MS))
        {
            apds9960_reset_counts(sensor);
            return gestureReceived;
        }
    }
};

// bool apds9960_gesture_valid(sensor){
//    uint8_t data = 1;
/* apds9960_dev_t *sens = (apds9960_dev_t *) sensor;
  i2c_bus_read_byte(sens->i2c_dev, GSTATUS, &data);
  sens->_gstatus_t.gfov = (data >> 1) & 0x01;
  sens->_gstatus_t.gvalid = data & 0x01;
  return sens->_gstatus_t.gvalid;*/
//    return true;

//}


// err_t apds9960_proximity_init(apds9960_t *sensor);
// err_t apds9960_read_raw_proximity(apds9960_t *sensor, uint8_t *proximity);
// err_t apds9960_read_proximity(apds9960_t *sensor, uint8_t *proximity);
// err_t apds9960_get_color_data(apds9960_t sensor, uint16_t *r,
// uint16_t *g, uint16_t *b, uint16_t *c);
// err_t apds9960_enable_color_engine(apds9960_t sensor, bool en);
// err_t apds9960_get_ambient_light(apds9960_t *sensor, uint16_t *l);
