#include <stdio.h>
#include "mpu6050.h"
#include "mpu6050_internals.h"
#include "error_module.h"
#include "time_module.h"


#define MPU6050_I2C_ADDRESS 0x38

typedef struct {
    uint8_t dev_addr;
    i2c_dev_t* i2c_dev_hadler;
    uint32_t i2c_clk;

    
} mpu6050_dev_t;



mpu6050_t * mpu6050_init(i2c_bus_t* i2c_bus)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *) malloc(sizeof(mpu6050_dev_t));
    if ((void *)sens == NULL || (void *)i2c_bus == NULL)
        return NULL;

    sens->dev_addr = MPU6050_I2C_ADDRESS; 
    sens->i2c_clk = MAX_CLK;

    i2c_dev_t* dev;
    dev = i2c_add_master_device( MPU6050_I2C_ADDRESS,  MAX_CLK, i2c_bus);
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
    if (sensor->i2c_dev_hadler ==NULL){
        printf("12c dev hadler no inicializado");
        return E_FAIL;
    }
    i2c_dev_t * handler = *sensor->i2c_dev_hadler;
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


err_t mpu6050_setup_default(mpu6050_t* sensor)
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

err_t mpu6050_set_acce_range(mpu6050_t *sensor, acel_range_t range)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
     // read first and do |=mode to preserve self-test?
    mpu6050_write(sens, ACCEL_CONFIG, (uint8_t)range, 1);
    return E_OK;
}


err_t mpu6050_set_gyro_range(mpu6050_t *sensor, gyro_range_t range){
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
     // read first and do |=mode to preserve self-test?
    mpu6050_write(sens, ACCEL_CONFIG, (uint8_t)range, 1);
    return E_OK;
}

err_t mpu6050_get_id(mpu6050_t *sensor){
     mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
     uint8_t *val = malloc(sizeof(uint8_t));
    mpu6050_read(sens,WHO_AM_I,val,1);
    return E_OK;
}


// err_t mpu6050_get_acce_raw(mpu6050_t *sensor,acce_raw *accel_data);
// err_t mpu6050_get_gyro_raw(mpu6050_t *sensor,gyro_raw *gyro_data);
// err_t mpu6050_get_acce_sensitivity(mpu6050_t *sensor,float *acce_sensitivity);
// err_t mpu6050_get_gyro_sensitivity(mpu6050_t *sensor,float *gyro_sensitivity);

// err_t mpu6050_get_acce_range(mpu6050_t *sensor, acel_range *range);
// err_t mpu6050_get_gyro_range(mpu6050_t *sensor, gyro_range *range);
// err_t mpu6050_get_gyro(mpu6050_t sensor, float *ax, float *ay, float *az);
// err_t mpu6050_get_vel(mpu6050_t sensor, float *gx, float *gy, float *gz);
// err_t mpu6050_get_orientation(mpu6050_t sensor, float *roll, float *pitch, float *yaw);
