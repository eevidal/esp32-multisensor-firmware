#include <stdio.h>
#include "mpu6050.h"
#include "mpu6050_internals.h"
#include "error_module.h"
#include "time_module.h"

typedef struct
{
    uint8_t dev_addr;
    i2c_dev_t *i2c_dev_hadler;
    uint32_t i2c_clk;
} mpu6050_dev_t;

mpu6050_t *mpu6050_init(i2c_bus_t *i2c_bus)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *)malloc(sizeof(mpu6050_dev_t));
    if ((void *)sens == NULL || (void *)i2c_bus == NULL)
        return NULL;

    sens->dev_addr = MPU6050_I2C_ADDRESS;
    sens->i2c_clk = MAX_CLK;

    i2c_dev_t *dev;
    dev = i2c_add_master_device(MPU6050_I2C_ADDRESS, MAX_CLK, i2c_bus);
    if (dev == NULL)
        printf("Algo salio mal seteando handler i2c\n");
    sens->i2c_dev_hadler = dev;

    return (mpu6050_t *)sens;
}

err_t mpu6050_delete(mpu6050_t *sensor)
{
    if (*sensor == NULL)
        return E_OK;
    mpu6050_dev_t *sens = (mpu6050_dev_t *)(*sensor);
    i2c_del_master_device(sens->i2c_dev_hadler);
    free(sens);
    *sensor = NULL;
    return E_OK;
}; // RNF.4

/**
 * Writes data to the specified register of the mpu6050 sensor.
 *
 * @param sensor Pointer to the mpu6050 sensor device.
 * @param addr Register address to write to.
 * @param buf Pointer to the data buffer to write.
 * @param len Number of bytes to write.
 * @return Error code returned by the underlying I2C write operation.
 */
err_t mpu6050_write(mpu6050_dev_t *sensor, uint8_t addr, uint8_t buf, uint8_t len)
{
    if (sensor->i2c_dev_hadler == NULL)
    {
        printf("12c dev hadler no inicializado");
        return E_FAIL;
    }
    i2c_dev_t *handler = *sensor->i2c_dev_hadler;
    return (i2c_write(handler, addr, buf, len));
}

/**
 * Reads data from the specified register of the mpu6050 sensor.
 *
 * @param sensor Pointer to the mpu6050 sensor device.
 * @param addr Register address to read from.
 * @param buf Pointer to the buffer to store the read data.
 * @param len Number of bytes to read.
 * @return Error code returned by the underlying I2C read operation.
 */
err_t mpu6050_read(mpu6050_dev_t *sensor, uint8_t addr, uint8_t *buf, uint8_t len)
{
    return (i2c_read(sensor->i2c_dev_hadler, addr, buf, len));
}

err_t mpu6050_setup_default(mpu6050_t *sensor)
{
    mpu6050_set_pwr_clock(sensor, CLK_PLL_XGYRO_C);
    mpu6050_set_acce_range(sensor, ACCE_2G);
    mpu6050_set_gyro_range(sensor, GYRO_250DPS);
    return E_OK;
}

err_t mpu6050_set_pwr_clock(mpu6050_t *sensor, mpu6050_pwr_clk_t mode)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;

    mpu6050_write(sens, PWR_MGMT_1, (uint8_t)mode, 1);
    return E_OK;
}

err_t mpu6050_set_dlpf(mpu6050_t *sensor, dlpf_t filter)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
    uint8_t config;
    mpu6050_read(sens, CONFIG, &config, 1);
    config &= 0b11111000;
    config |= filter;
    mpu6050_write(sens, CONFIG, config, 1);
    return E_OK;
}

err_t mpu6050_set_sample_rate(mpu6050_t *sensor, int16_t rate){
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
    if (rate < 4)
        rate = 4;
    if (rate > 1000)
        rate = 1000;
    uint8_t smprt_div;
    smprt_div = (1000 / rate ) - 1;
    mpu6050_write(sens, SMPLRT_DIV, smprt_div, 1);
    return E_OK;
}

err_t mpu6050_set_acce_range(mpu6050_t *sensor, acel_range_t range)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
    // read first and do |=mode to preserve self-test?
    mpu6050_write(sens, ACCEL_CONFIG, (uint8_t)range, 1);
    return E_OK;
}

err_t mpu6050_set_gyro_range(mpu6050_t *sensor, gyro_range_t range)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
    // read first and do |=mode to preserve self-test?
    mpu6050_write(sens, GYRO_CONFIG, (uint8_t)range, 1);
    return E_OK;
}

err_t mpu6050_get_id(mpu6050_t *sensor,uint8_t *val )
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
    mpu6050_read(sens, WHO_AM_I, val, 1);
    return E_OK;
}

uint16_t mpu_get_temp(mpu6050_dev_t *sensor_dev)
{
    uint8_t t_0;
    uint8_t t_1;
    mpu6050_read(sensor_dev, TEMP_OUT_H, &t_1, 1);
    mpu6050_read(sensor_dev, TEMP_OUT_L, &t_0, 1);
    uint16_t t = (((uint16_t)t_1) << 8) | (uint16_t)t_0;
    return t;
}

uint16_t mpu_get_acce_x(mpu6050_dev_t *sensor_dev)
{
    uint8_t x_0;
    uint8_t x_1;
    mpu6050_read(sensor_dev, ACCEL_XOUT_H, &x_1, 1);
    mpu6050_read(sensor_dev, ACCEL_XOUT_L, &x_0, 1);
    uint16_t x = (((uint16_t)x_1) << 8) | (uint16_t)x_0;
    return x;
}

uint16_t mpu_get_acce_y(mpu6050_dev_t *sensor_dev)
{
    uint8_t y_0;
    uint8_t y_1;
    mpu6050_read(sensor_dev, ACCEL_YOUT_H, &y_1, 1);
    mpu6050_read(sensor_dev, ACCEL_YOUT_L, &y_0, 1);
    uint16_t y = (((uint16_t)y_1) << 8) | (uint16_t)y_0;
    return y;
}

uint16_t mpu_get_acce_z(mpu6050_dev_t *sensor_dev)
{
    uint8_t z_0;
    uint8_t z_1;
    mpu6050_read(sensor_dev, ACCEL_ZOUT_H, &z_1, 1);
    mpu6050_read(sensor_dev, ACCEL_ZOUT_L, &z_0, 1);
    uint16_t z = (((uint16_t)z_1) << 8) | (uint16_t)z_0;
    return z;
}

err_t mpu6050_get_acce_raw(mpu6050_t *sensor, acce_raw_t *accel_data)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
    accel_data->x = mpu_get_acce_x(sens);
    accel_data->y = mpu_get_acce_y(sens);
    accel_data->z = mpu_get_acce_z(sens);
    return E_OK;
}

uint16_t mpu_get_gyro_x(mpu6050_dev_t *sensor_dev)
{
    uint8_t x_0;
    uint8_t x_1;
    mpu6050_read(sensor_dev, GYRO_XOUT_H, &x_1, 1);
    mpu6050_read(sensor_dev, GYRO_XOUT_L, &x_0, 1);
    uint16_t x = (((uint16_t)x_1) << 8) | (uint16_t)x_0;
    return x;
}

uint16_t mpu_get_gyro_y(mpu6050_dev_t *sensor_dev)
{
    uint8_t y_0;
    uint8_t y_1;
    mpu6050_read(sensor_dev, GYRO_YOUT_H, &y_1, 1);
    mpu6050_read(sensor_dev, GYRO_YOUT_L, &y_0, 1);
    uint16_t y = (((uint16_t)y_1) << 8) | (uint16_t)y_0;
    return y;
}

uint16_t mpu_get_gyro_z(mpu6050_dev_t *sensor_dev)
{
    uint8_t z_0;
    uint8_t z_1;
    mpu6050_read(sensor_dev, GYRO_ZOUT_H, &z_1, 1);
    mpu6050_read(sensor_dev, GYRO_ZOUT_L, &z_0, 1);
    uint16_t z = (((uint16_t)z_1) << 8) | (uint16_t)z_0;
    return z;
}

err_t mpu6050_get_gyro_raw(mpu6050_t *sensor, gyro_raw_t *gyro_data)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
    gyro_data->x = mpu_get_gyro_x(sens);
    gyro_data->y = mpu_get_gyro_y(sens);
    gyro_data->z = mpu_get_gyro_z(sens);
    return E_OK;
}
/* 
 For each full scale setting, the accelerometer' sensitivity per
  LSB in ACCE_xOUT is shown in the table below:
 
  <pre>
  FS_SEL | Full Scale Range   | LSB Sensitivity
  -------+--------------------+----------------
  0      | +/- 2g             | 16384 LSB/g
  1      | +/- 4g             | 8192 LSB/g
  2      | +/- 8g             | 4096 LSB/g
  3      | +/- 16g            | 2048 LSB/g
  
 */

err_t mpu6050_get_acce_sensitivity(mpu6050_t *sensor, float *acce_sensitivity)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;
    uint8_t acce_fs;
    mpu6050_read(sens, ACCEL_CONFIG, &acce_fs,1);
    acce_fs = (acce_fs >> 3) & 0x03;
    switch (acce_fs) {
    case ACCE_2G:
        *acce_sensitivity = 16384;
        break;
    case ACCE_4G:
        *acce_sensitivity = 8192;
        break;
    case ACCE_8G:
        *acce_sensitivity = 4096;
        break;
    case ACCE_16G:
        *acce_sensitivity = 2048;
        break;
    default:
        break;
    }
    return E_OK;
}

/* 
 For each full scale setting, the gyroscopes' sensitivity per
  LSB in GYRO_xOUT is shown in the table below:
 
  <pre>
  FS_SEL | Full Scale Range   | LSB Sensitivity
  -------+--------------------+----------------
  0      | +/- 250 degrees/s  | 131 LSB/deg/s
  1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
  2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
  3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
  
 */

// err_t mpu6050_get_gyro_sensitivity(mpu6050_t *sensor,float *gyro_sensitivity);
err_t mpu6050_get_gyro_sensitivity(mpu6050_t *sensor, float *gyro_sensitivity)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;
    uint8_t gyro_fs;
    mpu6050_read(sens, GYRO_CONFIG, &gyro_fs,1);
    gyro_fs = (gyro_fs >> 3) & 0x03;
    switch (gyro_fs) {
    case GYRO_250DPS:
        *gyro_sensitivity = 131;
        break;
    case GYRO_500DPS:
        *gyro_sensitivity = 65.5;
        break;
    case GYRO_1000DPS:
        *gyro_sensitivity = 32.8;
        break;
    case GYRO_2000DPS:
        *gyro_sensitivity = 16.4;
        break;
    default:
        break;
    }
    return E_OK;
}

// err_t mpu6050_get_acce_range(mpu6050_t *sensor, acel_range *range);
// err_t mpu6050_get_gyro_range(mpu6050_t *sensor, gyro_range *range);
// err_t mpu6050_get_gyro(mpu6050_t sensor, float *ax, float *ay, float *az);

// err_t mpu6050_get_vel(mpu6050_t sensor, float *gx, float *gy, float *gz);
// err_t mpu6050_get_orientation(mpu6050_t sensor, float *roll, float *pitch, float *yaw);
