#include <stdio.h>
#include "apds9960.h"
#include "apds9960_internals.h"
#include "error_module.h"
#include "time_module.h"

#define MAX_CLK 100000

typedef struct
{
    uint16_t dev_addr;
    i2c_dev_t *i2c_dev_hadler;
    uint32_t i2c_clk;
    uint64_t timeout;
    uint8_t enable;
    uint8_t gest_cnt;  /*< counter of gesture >*/
    uint8_t up_cnt;    /*< counter of up gesture >*/
    uint8_t down_cnt;  /*< counter of down gesture >*/
    uint8_t left_cnt;  /*< counter of left gesture >*/
    uint8_t right_cnt; /*< counter of right gesture >*/

} apds9960_dev_t;

apds9960_t *apds9960_create(i2c_bus_t *i2c_bus)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)malloc(sizeof(apds9960_dev_t));
    if ((void *)sens == NULL || (void *)i2c_bus == NULL)
        return NULL;
    sens->dev_addr = APDS9960_I2C_ADDRESS;
    sens->i2c_clk = MAX_CLK;
    i2c_dev_t *dev;
    dev = i2c_add_master_device(APDS9960_I2C_ADDRESS, MAX_CLK, i2c_bus);
    if (dev == NULL)
        return NULL;
    sens->i2c_dev_hadler = dev;
    sens->enable = 0;
    sens->up_cnt = 0;
    sens->down_cnt = 0;
    sens->left_cnt = 0;
    sens->right_cnt = 0;
    sens->gest_cnt = 0;
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
    if (sensor->i2c_dev_hadler == NULL)
    {
        printf("12c dev hadler no inicializado");
        return E_FAIL;
    }
    i2c_dev_t *handler = sensor->i2c_dev_hadler;
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
    apds9960_dev_t *sens = (apds9960_dev_t *)(sensor);
    switch (mode)
    {
    case APDS9960_POWER:
        sens->enable &= ~PON | 0x40;
        break;
    case APDS9960_ALS:
        sens->enable &= ~AEN | 0x40;
        break;
    case APDS9960_PROXIMIMTY:
        sens->enable &= ~PEN | 0x40;
        break;
    case APDS9960_WAIT:
        sens->enable &= ~WEN | 0x40;
        break;
    case APDS9960_AINT:
        sens->enable &= ~AIEN | 0x40;
        break;
    case APDS9960_PINT:
        sens->enable &= ~PIEN | 0x40;
        break;
    case APDS9960_GESTURE:
        sens->enable &= ~GEN | 0x40;
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

err_t apds9960_set_proximity_threshold(apds9960_t *sensor, uint8_t low, uint8_t high, uint8_t persistence)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    apds9960_write(sens, PILT, low, 1);
    apds9960_write(sens, PIHT, high, 1);
    if (persistence > 15)
        persistence = 15;
    uint8_t prev_pers;
    apds9960_read(sens, PERS, &prev_pers, 1);
    prev_pers |= (persistence << 4);
    persistence = prev_pers;
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
    uint8_t prev_again;
    apds9960_read(sens, CONTROL, &prev_again, 1);
    again |= prev_again;
    return (apds9960_write(sens, CONTROL, again, 1));
}

err_t apds9960_set_pgain(apds9960_t *sensor, apds9960_gain_t pgain)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t temp = pgain << 2;
    uint8_t prev_pgain;
    apds9960_read(sens, CONTROL, &prev_pgain, 1);
    temp |= prev_pgain;
    return (apds9960_write(sens, CONTROL, temp, 1));
};

err_t apds9960_set_ldrive(apds9960_t *sensor, apds9960_ldrive_t ldrive)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t temp = ldrive << 6;
    uint8_t prev_control;
    apds9960_read(sens, CONTROL, &prev_control, 1);
    temp |= prev_control;
    return (apds9960_write(sens, CONTROL, temp, 1));
};

// CONFIG2
err_t apds9960_set_ggain(apds9960_t *sensor, apds9960_gain_t ggain)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t temp = ggain << 5;
    uint8_t prev_gconf2;
    apds9960_read(sens, GCONF2, &prev_gconf2, 1);
    temp |= prev_gconf2;
    return (apds9960_write(sens, GCONF2, temp, 1));
};

err_t apds9960_set_gldrive(apds9960_t *sensor, apds9960_ldrive_t gldrive)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t temp = gldrive << 3;
    uint8_t prev_conf;
    apds9960_read(sens, GCONF2, &prev_conf, 1);
    temp |= prev_conf;
    return (apds9960_write(sens, GCONF2, temp, 1));
};

err_t apds9960_set_gwtime(apds9960_t *sensor, apds9960_gwtime_t gwtime)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t temp = 0x07 & gwtime;
    uint8_t prev_conf;
    apds9960_read(sens, GCONF2, &prev_conf, 1);
    temp |= prev_conf;
    return (apds9960_write(sens, GCONF2, temp, 1));
};

// WTIME
err_t apds9960_set_wtime(apds9960_t *sensor, uint8_t wtime)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t val = 256 - wtime / 2.78;
    return (apds9960_write(sens, WTIME, val, 1));
}

// ATIME
err_t apds9960_set_atime(apds9960_t *sensor, uint8_t atime)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    if (sens == NULL)
    {
        printf("sensor no inicializado en atime\n");
        return E_FAIL;
    }
    uint8_t val = 256 - atime / 2.78;
    return (apds9960_write(sens, ATIME, val, 1));
}

// WLONG
err_t apds9960_enable_wlong(apds9960_t *sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t config1 = 0x61;
    return (apds9960_write(sens, CONFIG1, config1, 1));
}
err_t apds9960_disable_wlong(apds9960_t *sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t config1 = 0x60;
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
    tmp = pulses | (uint8_t)len;
    return (apds9960_write(sens, PPULSE, tmp, 1));
}

// CONFIG2
err_t apds9960_set_proximity_sat_int_(apds9960_t *sensor, apds9960_psat_int_t sat)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t config2 = 0x00;
    uint8_t prev_conf;
    apds9960_read(sens, CONFIG2, &prev_conf, 1);
    config2 = sat | prev_conf;
    return (apds9960_write(sens, CONFIG2, config2, 1));
}

err_t apds9960_set_proximity_clear_int_(apds9960_t *sensor, apds9960_pclear_int_t clear)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t config2 = 0x00;
    uint8_t prev_conf;
    apds9960_read(sens, CONFIG2, &prev_conf, 1);
    config2 = clear | prev_conf;
    return (apds9960_write(sens, CONFIG2, config2, 1));
}

err_t apds9960_set_ledboost(apds9960_t *sensor, apds9960_ledboost_t boost)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t config2 = 0x00;
    uint8_t prev_conf;
    apds9960_read(sens, CONFIG2, &prev_conf, 1);
    config2 = boost | prev_conf;
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
    uint8_t prev_config3;
    apds9960_read(sens, CONFIG3, &prev_config3, 1);
    prev_config3 |= pcmd_val;
    return (apds9960_write(sens, CONFIG3, prev_config3, 1));
}

err_t apds9960_set_proximity_sai(apds9960_t *sensor, apds9960_psai_t sai_val)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t prev_config3;
    apds9960_read(sens, CONFIG3, &prev_config3, 1);
    prev_config3 |= sai_val;
    return (apds9960_write(sens, CONFIG3, prev_config3, 1));
}

err_t apds9960_set_proximity_mask(apds9960_t *sensor, apds9960_pmask_t mask)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t prev_config3;
    apds9960_read(sens, CONFIG3, &prev_config3, 1);
    prev_config3 |= mask;
    return (apds9960_write(sens, CONFIG3, prev_config3, 1));
}
// GCONFIG1

err_t apds9960_set_gestrure_gexmsk(apds9960_t *sensor, apds9960_gexmsk_t mask)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t prev_gconfig;
    apds9960_read(sens, GCONF1, &prev_gconfig, 1);
    prev_gconfig |= mask;
    return (apds9960_write(sens, GCONF1, prev_gconfig, 1));
}

err_t apds9960_set_gestrure_fifoth(apds9960_t *sensor, apds9960_gfifoth_t fifoth)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t prev_gconfig;
    apds9960_read(sens, GCONF1, &prev_gconfig, 1);
    prev_gconfig |= fifoth;
    return (apds9960_write(sens, GCONF1, prev_gconfig, 1));
}
err_t apds9960_set_gestrure_gexpers(apds9960_t *sensor, apds9960_gexpers_t gexper)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t prev_gconfig;
    apds9960_read(sens, GCONF1, &prev_gconfig, 1);
    prev_gconfig |= gexper;
    return (apds9960_write(sens, GCONF1, prev_gconfig, 1));
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
    return (apds9960_write(sens, GPULSE, tmp, 1));
}

// GCONF3
err_t apds9960_set_gesture_gdims(apds9960_t *sensor, apds9960_gdims_t gdim)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t prev_gconfig;
    apds9960_read(sens, GCONF3, &prev_gconfig, 1);
    prev_gconfig |= gdim;
    return (apds9960_write(sens, GCONF3, prev_gconfig, 1));
}

err_t apds9960_set_gesture_int_on(apds9960_t *sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t prev_gconfig;
    apds9960_read(sens, GCONF4, &prev_gconfig, 1);
    prev_gconfig |= 0b00000010;
    return (apds9960_write(sens, GCONF4, prev_gconfig, 1));
}

err_t apds9960_set_gesture_int_off(apds9960_t *sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t prev_gconfig;
    apds9960_read(sens, GCONF4, &prev_gconfig, 1);
    prev_gconfig &= 0b00000001; // preserve gmode
    return (apds9960_write(sens, GCONF4, prev_gconfig, 1));
}

err_t apds9960_set_gesture_gmode_on(apds9960_t *sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t prev_gconfig;
    apds9960_read(sens, GCONF4, &prev_gconfig, 1);
    prev_gconfig |= 0b00000001;
    return (apds9960_write(sens, GCONF4, prev_gconfig, 1));
}
err_t apds9960_set_gesture_gmode_off(apds9960_t *sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t prev_gconfig;
    apds9960_read(sens, GCONF4, &prev_gconfig, 1);
    prev_gconfig &= 0b00000010; // preserve gien
    return (apds9960_write(sens, GCONF4, prev_gconfig, 1));
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

err_t apds9960_gesture_valid(apds9960_t *sensor, uint8_t *data)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    apds9960_read(sens, GSTATUS, data, 1);
    return E_OK;
}

err_t apds9960_get_status(apds9960_t *sensor, uint8_t *data)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    apds9960_read(sens, ENABLE, data, 1);
    return E_OK;
}

err_t apds9960_get_id(apds9960_t *sensor, uint8_t *val)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    printf("get id \n");
    apds9960_read(sens, ID, val, 1);
    return E_OK;
}

void apds9960_reset_counts(apds9960_t *sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->up_cnt = 0;
    sens->down_cnt = 0;
    sens->left_cnt = 0;
    sens->right_cnt = 0;
    sens->gest_cnt = 0;
}

err_t apds9960_read_gesture(apds9960_t *sensor, uint8_t *gesture)
{

    uint8_t cant = 0;
    uint8_t buf[4];
    uint64_t t;

    uint8_t valid = 0;
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    if (sensor == NULL)
    {
        printf("Sensor no inicializado");
        return E_FAIL;
    }
    while (1)
    {
        int up_down_diff = 0;
        int left_right_diff = 0;
        apds9960_gesture_valid(sensor, &valid);
        if (!valid)
        {
            printf("gesture no valid\n");
            *gesture = FAR;
            return E_OK;
        }
        delay_us(30);
        apds9960_read(sens, GFLVL, &cant, 1);
        apds9960_read(sens, GFIFO_U, buf, 4); // page read

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
            *gesture = NONE;
            return E_OK;
        }
        if (up_down_diff != 0 || left_right_diff != 0)
            t = now();
        if (*gesture || elapsed_time(t) > sens->timeout)
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

err_t apds9960_get_color_data(apds9960_t *sensor, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
    apds9960_read_color(sensor, CDATA, c);
    apds9960_read_color(sensor, RDATA, r);
    apds9960_read_color(sensor, GDATA, g);
    apds9960_read_color(sensor, BDATA, b);
    return E_OK;
}

void apds9960_diagnose(apds9960_t *sensor)
{

    uint8_t val;
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    for (uint8_t reg = 0x80; reg <= 0xAF; reg++)
    {
        if ((reg != 0x82) &&
            (reg != 0x88) &&
            (reg != 0x8A) &&
            (reg != 0x91) &&
            (reg != 0xA8) &&
            (reg != 0xAC) &&
            (reg != 0xAD))
        {
            i2c_read(sens->i2c_dev_hadler, reg, &val, 1);
            printf("Register 0x%X value 0x%X \n", reg, val);
            val = 0x00;
        }
        else
            printf("Avoid Register 0x%X\n", reg);
    }
}

err_t apds9960_init(apds9960_t *sensor)
{
    /*disable all features*/
    apds9960_disable_engine(sensor, APDS9960_ALL);
    /* Set default values for ambient light and proximity registers */
    apds9960_set_atime(sensor, 103);
    apds9960_set_wtime(sensor, 27);
    apds9960_set_proximity_pulse(sensor, APDS9960_LEN_16US, 7);
    apds9960_set_poffset_ur(sensor, 0);
    apds9960_set_poffset_dl(sensor, 0);
    // config1
    apds9960_disable_wlong(sensor);
    apds9960_set_proximity_threshold(sensor, 0, 50, 2);
    apds9960_set_proximity_sat_int_(sensor, APDS9960_PSAT_OFF);
    apds9960_set_ldrive(sensor, APDS9960_LDRIVE_100MA);
    apds9960_set_pgain(sensor, APDS9960_GAIN_4X);
    apds9960_set_again(sensor, APDS9960_AGAIN_4X);
    apds9960_set_als_threshold(sensor, 0, 0xFFFF);
    // config2
    apds9960_set_ledboost(sensor, APDS9960_LBOOST_100P);
    apds9960_set_proximity_sat_int_(sensor, APDS9960_PSAT_OFF);
    apds9960_set_als_clear_int(sensor, APDS9960_PCLEAR_OFF);
    // config3
    apds9960_set_proximity_pcmp(sensor, APDS9960_PCMP_OFF);
    apds9960_set_proximity_sai(sensor, APDS9960_PSAI_OFF);
    apds9960_set_proximity_mask(sensor, APDS9960_PMASK_ALL);
    /* Set default values for gesture sense registers */
    apds9960_set_gesture_threshold(sensor, 40, 30);
    // gconfig1
    apds9960_set_gestrure_fifoth(sensor, APDS9960_GFIFOTH_4);
    apds9960_set_gestrure_gexmsk(sensor, APDS9960_GEXMSK_ALL);
    apds9960_set_gestrure_gexpers(sensor, APDS9960_GEXPERS_1);
    // gconfig2
    apds9960_set_ggain(sensor, APDS9960_GAIN_4X);
    apds9960_set_gldrive(sensor, APDS9960_LDRIVE_100MA);
    apds9960_set_gwtime(sensor, APDS9960_GWTIME_2_8MS);

    apds9960_set_gestrure_offsets(sensor, 0, 0, 0, 0);
    apds9960_set_gesture_pulse(sensor, APDS9960_LEN_16US, 10);
    // gconfig3
    apds9960_set_gesture_gdims(sensor, APDS9960_GDIM_ALL);
    // gconfig4
    apds9960_set_gesture_int_off(sensor);
    apds9960_set_gesture_gmode_off(sensor);

    return E_OK;
}

/* Enable gesture mode
   Set ENABLE to 0 (power off)
   Set GWTIME to 0xFF
   Set AUX to LED_BOOST_300
   Enable PON, WEN, PEN, GEN in ENABLE
*/
err_t apds9960_gesture_enable(apds9960_t *sensor)
{
    apds9960_reset_counts(sensor);
    apds9960_set_wtime(sensor, 0xFF);
    apds9960_set_gesture_pulse(sensor, APDS9960_LEN_16US, 10);
    apds9960_set_ledboost(sensor, APDS9960_LBOOST_300P);
    apds9960_set_gesture_int_off(sensor); // no interruptions
    apds9960_set_gesture_gmode_on(sensor);
    apds9960_enable_engine(sensor, APDS9960_POWER);
    apds9960_enable_engine(sensor, APDS9960_WAIT);
    apds9960_enable_engine(sensor, APDS9960_PROXIMIMTY);
    apds9960_enable_engine(sensor, APDS9960_GESTURE);
    return E_OK;
}

err_t apds9960_gesture_disable(apds9960_t *sensor)
{
    apds9960_set_gesture_gmode_off(sensor);
    apds9960_disable_engine(sensor, APDS9960_GESTURE);
    return E_OK;
}

err_t apds9960_color_enable(apds9960_t *sensor)
{
    /* Set default gain, interrupts, enable power, and enable sensor */
    apds9960_set_again(sensor, APDS9960_AGAIN_4X);
    apds9960_disable_engine(sensor, APDS9960_AINT);
    apds9960_enable_engine(sensor, APDS9960_POWER);
    apds9960_enable_engine(sensor, APDS9960_ALS);
    return E_OK;
}

err_t apds9960_color_disable(apds9960_t *sensor)
{
    apds9960_disable_engine(sensor, APDS9960_ALS);
    return E_OK;
}

err_t apds9960_proximity_enable(apds9960_t *sensor)
{
    /* Set default gain, LED, interrupts, enable power, and enable sensor */
    apds9960_set_pgain(sensor, APDS9960_GAIN_4X);
    apds9960_set_ldrive(sensor, APDS9960_LDRIVE_100MA);
    apds9960_disable_engine(sensor, APDS9960_PINT);
    apds9960_enable_engine(sensor, APDS9960_POWER);
    apds9960_enable_engine(sensor, APDS9960_PROXIMIMTY);
    return E_OK;
}

err_t apds9960_proximity_disable(apds9960_t *sensor)
{
    apds9960_disable_engine(sensor, APDS9960_PROXIMIMTY);
    return E_OK;
}

// Private Functions

/**
 * Reads color data from the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param data Specifies the color data to read (CDATA, RDATA, GDATA, or BDATA).
 * @param color_data Pointer to store the read color data.
 * @return E_OK on success.
 */
err_t apds9960_read_color(apds9960_t *sensor, uint8_t data, uint16_t *color_data)
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

err_t apds9960_calculate_lux(apds9960_t sensor, uint16_t r,
                             uint16_t g, uint16_t b, uint16_t *l);
