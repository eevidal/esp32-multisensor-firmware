#include <stdio.h>
#include "apds9960.h"
#include "apds9960_internals.h"
#include "error_module.h"
#include "time_module.h"

#define APDS9960_I2C_ADDRESS (0x39)
#define MAX_CLK 400000
#define ID 0x92


typedef struct
{
    uint8_t dev_addr;
    i2c_dev_t *i2c_dev_hadler;
    uint32_t i2c_clk;
    uint64_t timeout;
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
 //   uint8_t wtime;    /**0x8D*/
 //   uint8_t atime;    /**0x8C*/
    uint8_t ppulse;   /**0x8E*/
    uint8_t gpulse;   /**0xA6*/
    uint8_t gmode;    /**0xAB<0>*/
    uint8_t up_cnt;    /*< counter of up gesture >*/
    uint8_t down_cnt;  /*< counter of down gesture >*/
    uint8_t left_cnt;  /*< counter of left gesture >*/
    uint8_t right_cnt; /*< counter of right gesture >*/

} apds9960_dev_t;

apds9960_t *apds9960_init(i2c_bus_t *i2c_bus)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)malloc(sizeof(apds9960_dev_t));
    if ((void *)sens == NULL || (void *)i2c_bus == NULL)
        return NULL;
    sens->dev_addr = APDS9960_I2C_ADDRESS;
    sens->i2c_clk = MAX_CLK;
    i2c_dev_t* dev;
    dev = i2c_add_master_device( APDS9960_I2C_ADDRESS,  MAX_CLK, i2c_bus);
    if (dev == NULL)
        printf("Algo salio mal seteando handler i2c\n");
    
    sens->i2c_dev_hadler = dev;
    sens->enable = 0x00;
    sens->status = 0x00;
    sens->control = 0x00;
    sens->gstatus = 0x00;
    sens->config1 = 0x60; // Bit 6 and 5 are reserved, and are automatically set to 1 at POR.
    sens->config2 = 0x01;
    sens->config3 = 0x00;
    sens->gconfig1 = 0x00;
    sens->gconfig2 = 0x00;
    sens->gconfig3 = 0x00;
    sens->gconfig4 = 0x00;
    sens->pers = 0x00;
    sens->ppulse = 0x40;
    sens->gmode = 0x00;
    sens->up_cnt = 0x00;
    sens->down_cnt = 0x00;
    sens->left_cnt = 0x00;
    sens->right_cnt = 0x00;

    return (apds9960_t)sens;
}

err_t apds9960_delete(apds9960_t *sensor)
{
    if (*sensor == NULL)
        return E_OK;

    apds9960_dev_t *sens = (apds9960_dev_t *)(*sensor);
    i2c_del_master_device(sens->i2c_dev_hadler);
    free(sens);
    *sensor = NULL;
    return E_OK;
}; // RNF.4

/**
 * Writes data to the specified register of the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param addr Register address to write to.
 * @param buf Pointer to the data buffer to write.
 * @param len Number of bytes to write.
 * @return Error code returned by the underlying I2C write operation.
 */
err_t apds9960_write(apds9960_dev_t *sensor, uint8_t addr, uint8_t buf, uint8_t len)
{
    if (sensor->i2c_dev_hadler ==NULL){
        printf("12c dev hadler no inicializado");
        return E_FAIL;
    }
    i2c_dev_t * handler = *sensor->i2c_dev_hadler;
    return (i2c_write(handler, addr, buf, len));
}

/**
 * Reads data from the specified register of the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param addr Register address to read from.
 * @param buf Pointer to the buffer to store the read data.
 * @param len Number of bytes to read.
 * @return Error code returned by the underlying I2C read operation.
 */
err_t apds9960_read(apds9960_dev_t *sensor, uint8_t addr, uint8_t *buf, uint8_t len)
{
    return (i2c_read(sensor->i2c_dev_hadler, addr, buf, len));
}


err_t apds9960_set_timeout(apds9960_t *sensor, uint64_t timeout)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->timeout = timeout;
    return E_OK;
}

err_t apds9960_enable_engine(apds9960_t *sensor, apds9960_mode_t mode)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
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
    apds9960_write(sens, ENABLE, data, 1);
    apds9960_reset_counts(sensor);
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
    apds9960_write(sens, ENABLE, data, 1);
    return E_OK;
}

err_t apds9960_set_als_threshold(apds9960_t *sensor, uint16_t low, uint16_t high)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    apds9960_write(sens, AILTL, (uint8_t)(low & 0xFF), 1);
    apds9960_write(sens, AILTH, (uint8_t)(low >> 8), 1);
    apds9960_write(sens, AIHTL, (uint8_t)(high & 0xFF), 1);
    apds9960_write(sens, AIHTH, (uint8_t)(high >> 8), 1);
    return E_OK;
}

err_t apds9960_set_proximity_threshold(apds9960_t *sensor, uint8_t threshold, uint8_t persistence)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    apds9960_write(sens, PILT, (threshold & 0xFF), 1);
    apds9960_write(sens, PIHT, (threshold >> 8), 1);
    if (persistence > 7)
        persistence = 7;
    sens->pers &= 0x0F;
    sens->pers |= (persistence << 4);
    apds9960_write(sens, PERS, persistence, 1);
    return E_OK;
}

err_t apds9960_set_gesture_threshold(apds9960_t *sensor, uint8_t gpenth, uint8_t gexth)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    apds9960_write(sens, GPENTH, gpenth, 1);
    apds9960_write(sens, GEXTH, gexth, 1);
    return E_OK;
}


// CONTROL Register

err_t apds9960_set_again(apds9960_t *sensor, apds9960_again_t again)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t temp = 0x03 & again;

    temp |= sens->control;
    sens->control = temp;
    return (apds9960_write(sens, CONTROL, temp, 1));
}

err_t apds9960_set_pgain(apds9960_t *sensor, apds9960_gain_t pgain)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t temp = 0x0C & pgain;
    sens->control |= temp;
    temp = sens->control;
    return (apds9960_write(sens, CONTROL, temp, 1));
};

err_t apds9960_set_ldrive(apds9960_t *sensor, apds9960_ldrive_t ldrive)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t temp = 0xC0 & ldrive;
    sens->control |= temp;
    temp = sens->control;
    return (apds9960_write(sens, CONTROL, temp, 1));
};

// CONFIG2
err_t apds9960_set_ggain(apds9960_t *sensor, apds9960_gain_t ggain)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t temp = 0x60 & ggain;
    sens->gconfig2 |= temp;
    temp = sens->gconfig2;
    return (apds9960_write(sens, GCONF2, temp, 1));
};

err_t apds9960_set_gldrive(apds9960_t *sensor, apds9960_ldrive_t gldrive)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t temp = 0x14 & gldrive;
    sens->gconfig2 |= temp;
    temp = sens->gconfig2;
    return (apds9960_write(sens, GCONF2, temp, 1));
};

err_t apds9960_set_gwtime(apds9960_t *sensor, apds9960_gwtime_t gwtime)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t temp = 0x07 & gwtime;
    sens->gconfig2 |= temp;
    temp = sens->gconfig2;
    return (apds9960_write(sens, GCONF2, temp, 1));
};

// WTIME
err_t apds9960_set_wtime(apds9960_t *sensor, uint8_t wtime)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;

    return (apds9960_write(sens, WTIME, wtime, 1));
}

// ATIME
err_t apds9960_set_atime(apds9960_t *sensor, uint8_t atime)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    if (sens==NULL){
        printf("sensor no inicializado en atime\n");
        return E_FAIL;
    }
    return (apds9960_write(sens, ATIME, atime, 1));
}

// WLONG
err_t apds9960_enable_wlong(apds9960_t *sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t config1 = 0x61;
    sens->config1 = config1;
    return (apds9960_write(sens, CONFIG1, config1, 1));
}
err_t apds9960_disable_wlong(apds9960_t *sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t config1 = 0x60;
    sens->config1 = config1;
    return (apds9960_write(sens, CONFIG1, config1, 1));
}

// PPULSE
err_t apds9960_set_proximity_pulse(apds9960_t *sensor, apds9960_pulse_len_t len, uint8_t pulses)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t tmp = 0;
    if (pulses < 1)
        pulses = 1;
    if (pulses > 64)
        pulses = 64;
    pulses--;
    sens->ppulse = pulses | (uint8_t)len;
    tmp = sens->ppulse;
    return (apds9960_write(sens, GPULSE, tmp, 1));
}

// CONFIG2
err_t apds9960_set_proximity_sat_int_(apds9960_t *sensor, apds9960_psat_int_t sat)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t config2 = 0x00;
    config2 |= sat;
    sens->config2 = config2;
    return (apds9960_write(sens, CONFIG2, config2, 1));
}

err_t apds9960_set_proximity_clear_int_(apds9960_t *sensor, apds9960_pclear_int_t clear)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t config2 = 0x00;
    config2 |= clear;
    sens->config2 = config2;
    return (apds9960_write(sens, CONFIG2, config2, 1));
}

err_t apds9960_set_ledboost(apds9960_t *sensor, apds9960_ledboost_t boost)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t config2 = 0x00;
    config2 |= boost;
    sens->config2 = config2;
    return (apds9960_write(sens, CONFIG2, config2, 1));
}

// P_OFFSETS
err_t apds9960_set_poffset_ur(apds9960_t *sensor, uint8_t offset)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return (apds9960_write(sens, POFFSET_UR, offset, 1));
}
err_t apds9960_set_poffset_dl(apds9960_t *sensor, uint8_t offset)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return (apds9960_write(sens, POFFSET_DL, offset, 1));
}

// CONFIG3
err_t apds9960_set_proximity_pcmp(apds9960_t *sensor, apds9960_pcmp_t pcmd_val)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->config3 |= pcmd_val;
    uint8_t config3 = sens->config3;
    return (apds9960_write(sens, CONFIG3, config3, 1));
}

err_t apds9960_set_proximity_sai(apds9960_t *sensor, apds9960_psai_t sai_val)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->config3 |= sai_val;
    uint8_t config3 = sens->config3;
    return (apds9960_write(sens, CONFIG3, config3, 1));
}

err_t apds9960_set_proximity_mask(apds9960_t *sensor, apds9960_pmask_t mask)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->config3 |= mask;
    uint8_t config3 = sens->config3;
    return (apds9960_write(sens, CONFIG3, config3, 1));
}
// GCONFIG1

err_t apds9960_set_gestrure_gexmsk(apds9960_t *sensor, apds9960_gexmsk_t mask)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->gconfig1 |= mask;
    uint8_t gconfig1 = sens->gconfig1;
    return (apds9960_write(sens, GCONF1, gconfig1, 1));
}

err_t apds9960_set_gestrure_fifoth(apds9960_t *sensor, apds9960_gfifoth_t fifoth)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->gconfig1 |= fifoth;
    uint8_t gconfig1 = sens->gconfig1;
    return (apds9960_write(sens, GCONF1, gconfig1, 1));
}
err_t apds9960_set_gestrure_gexpers(apds9960_t *sensor, apds9960_gexpers_t gexper)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->gconfig1 |= gexper;
    uint8_t gconfig1 = sens->gconfig1;
    return (apds9960_write(sens, GCONF1, gconfig1, 1));
}

// CONFIG GOFFSET
err_t apds9960_set_gestrure_offsets(apds9960_t *sensor, uint8_t offset_up, uint8_t offset_down, uint8_t offset_left, uint8_t offset_right)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    apds9960_write(sens, GOFFSET_D, offset_down, 1);
    apds9960_write(sens, GOFFSET_U, offset_up, 1);
    apds9960_write(sens, GOFFSET_L, offset_left, 1);
    apds9960_write(sens, GOFFSET_R, offset_right, 1);
    return E_OK;
}

// GPULSE
err_t apds9960_set_gesture_pulse(apds9960_t *sensor, apds9960_pulse_len_t len, uint8_t pulses)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t tmp = 0;
    if (pulses < 1)
        pulses = 1;
    if (pulses > 64)
        pulses = 64;
    pulses--;
    tmp = pulses | (uint8_t)len;
    sens->gpulse = tmp;
    return (apds9960_write(sens, GPULSE, tmp, 1));
}

// GCONF3
err_t apds9960_set_gesture_gdims(apds9960_t *sensor, apds9960_gdims_t gdim)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->gconfig3 |= gdim;
    uint8_t gconfig3 = sens->gconfig3;
    return (apds9960_write(sens, GCONF3, gconfig3, 1));
}

err_t apds9960_set_gesture_int_on(apds9960_t *sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->gconfig4 |= 0b00000010;
    uint8_t gconfig4 = sens->gconfig4;
    return (apds9960_write(sens, GCONF4, gconfig4, 1));
}

err_t apds9960_set_gesture_int_off(apds9960_t *sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->gconfig4 &= 0b11111101;
    uint8_t gconfig4 = sens->gconfig4;
    return (apds9960_write(sens, GCONF4, gconfig4, 1));
}

// INTERRUPS
err_t apds9960_set_force_interrupt(apds9960_t *sensor, uint8_t iforce)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return (apds9960_write(sens, IFORCE, iforce, 1));
}
err_t apds9960_set_proximity_clear_int(apds9960_t *sensor, uint8_t piclear)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return (apds9960_write(sens, PICLEAR, piclear, 1));
}
err_t apds9960_set_color_clear_int(apds9960_t *sensor, uint8_t ciclear)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return (apds9960_write(sens, CICLEAR, ciclear, 1));
}
err_t apds9960_set_als_clear_int(apds9960_t *sensor, uint8_t aiclear)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return (apds9960_write(sens, AICLEAR, aiclear, 1));
}

uint8_t apds9960_gesture_valid(apds9960_t *sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t data = 0;
    apds9960_read(sens, GSTATUS, &data, 1);
    sens->gstatus = data;
    return data;
}

uint8_t apds9960_id(apds9960_t *sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t data = 0;
    apds9960_read(sens, ID, &data, 1);
    printf("ID %d\n",data);
    return data;
}


void apds9960_reset_counts(apds9960_t *sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->up_cnt = 0;
    sens->down_cnt = 0;
    sens->left_cnt = 0;
    sens->right_cnt = 0;
}

err_t apds9960_read_gesture(apds9960_t *sensor, apds9960_gesture_t *gesture)
{
    uint8_t cant;
    uint8_t buf[4];
     uint64_t t;
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    if (sensor == NULL){ 
        printf("Sensor no inicializado");
        return E_FAIL;
    }
    while (1)
    {
        int up_down_diff = 0;
        int left_right_diff = 0;
        gesture = NONE;
        apds9960_id(sensor);
        if (!apds9960_gesture_valid(sensor))
        {
            gesture = NONE;
            printf("NONE");
            return E_OK;
        }
        delay_us(30);
        apds9960_read(sens, GFLVL, &cant, 1);
        apds9960_read(sens, GFIFO_U, buf, 4); // page read
        printf("GFLV, %d\n", cant);
        printf("GFIFO, %d\n", (int)buf);

        if (abs((int)buf[0] - (int)buf[1]) > 13)
            up_down_diff += (int)buf[0] - (int)buf[1];

        if (abs((int)buf[2] - (int)buf[3]) > 13)
            left_right_diff += (int)buf[2] - (int)buf[3];

        if (up_down_diff != 0)
        {
            if (up_down_diff < 0)
                (sens->down_cnt > 0) ? *gesture = UP : sens->up_cnt++;
            else
                (sens->up_cnt > 0) ? *gesture = DOWN : sens->down_cnt++;
        }

        if (left_right_diff != 0)
        {
            if (left_right_diff < 0)
                (sens->right_cnt > 0) ? *gesture = LEFT : sens->left_cnt++;
            else // if (left_right_diff > 0)
                (sens->left_cnt > 0) ? *gesture = RIGHT : sens->right_cnt++;
        }
        if (up_down_diff == 0 && left_right_diff == 0)
        {
            gesture = NONE;
            return E_OK;
        }
        if (up_down_diff != 0 || left_right_diff != 0)
            t = now();
        if (gesture || elapsed_time(t) > sens->timeout)
        {
            apds9960_reset_counts(sensor);
            return E_OK;
        }
    }
};

err_t apds9960_read_raw_proximity(apds9960_t *sensor, uint8_t *proximity)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return apds9960_read(sens, PDATA, proximity, 1);
}


err_t apds9960_read_color(apds9960_t *sensor, uint8_t data,  uint16_t *color_data)
{
    uint8_t read_data[2];
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    switch (data)
    {
    case CDATA:
        apds9960_read(sens, CDATAL, &read_data[0], 1);
        apds9960_read(sens, CDATAH, &read_data[1], 1);
        break;
    case RDATA:
        apds9960_read(sens, RDATAL, &read_data[0], 1);
        apds9960_read(sens, RDATAH, &read_data[1], 1);
        break;
    case GDATA:
        apds9960_read(sens, GDATAL, &read_data[0], 1);
        apds9960_read(sens, GDATAH, &read_data[1], 1);
        break;
    case BDATA:
        apds9960_read(sens, BDATAL, &read_data[0], 1);
        apds9960_read(sens, BDATAH, &read_data[1], 1);
        break;
    default:
        break;
    } 
    uint16_t aux = 0x0000 | (uint16_t)read_data[1];
    *color_data = (aux << 8) | (uint16_t)read_data[0];

    return E_OK;
}

err_t apds9960_get_color_data(apds9960_t *sensor, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
    apds9960_read_color(sensor, CDATA, c);
    apds9960_read_color(sensor, RDATA, r);
    apds9960_read_color(sensor, GDATA, g);
    apds9960_read_color(sensor, BDATA, b);
    return E_OK;
}

err_t apds9960_gesture_init(apds9960_t *sensor)
{
    /* Set default values for ambient light and proximity registers */ 
    apds9960_set_atime(sensor, 10); 
    apds9960_set_again(sensor, APDS9960_AGAIN_4X);
        printf("Disable Engine");
    apds9960_disable_engine(sensor, APDS9960_GESTURE);
    apds9960_disable_engine(sensor, APDS9960_PROXIMIMTY);
    apds9960_disable_engine(sensor, APDS9960_ALS);
    apds9960_disable_engine(sensor, APDS9960_AINT);
    apds9960_disable_engine(sensor, APDS9960_PINT);
      printf("ALL Disable Engine");
    apds9960_set_als_clear_int(sensor, AICLEAR);
    apds9960_enable_engine(sensor, APDS9960_POWER);
    apds9960_set_gesture_gdims(sensor, APDS9960_GDIM_ALL); 
    apds9960_set_gestrure_fifoth(sensor, APDS9960_GFIFOTH_4);
    apds9960_set_ggain(sensor,APDS9960_GAIN_4X); 
    apds9960_set_gesture_threshold(sensor, 50,0); 
    apds9960_reset_counts(sensor);
    apds9960_set_ldrive(sensor, APDS9960_LDRIVE_100MA);
    apds9960_set_ledboost(sensor, APDS9960_LBOOST_100P);
    apds9960_set_gwtime(sensor,APDS9960_GWTIME_2_8MS);
    apds9960_set_gesture_pulse(sensor, APDS9960_LEN_32US, 8);
    apds9960_enable_engine(sensor, APDS9960_PROXIMIMTY);
    apds9960_enable_engine(sensor, APDS9960_GESTURE);
    return E_OK;
}
// Private Functions

err_t apds9960_calculate_lux(apds9960_t sensor, uint16_t r,
                             uint16_t g, uint16_t b, uint16_t *l);