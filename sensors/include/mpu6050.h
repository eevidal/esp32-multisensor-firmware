#ifndef _MPU_6050_H_
#define _MPU_6050_H_

#include "error_module.h"
#include "i2c_module.h"

typedef void* mpu6050_t;

typedef enum{

} acel_range;

typedef enum{

} gyro_range;

typedef struct {

} acce_raw;

typedef struct{
    
} gyro_raw;

typedef struct {
    float x;
    float y;
    float z;
} mpu6050_acce_t;

typedef struct {
    float x;
    float y;
    float z;
} mpu6050_gyro_t;


typedef struct{

} imu_t;

typedef void * mpu6050_t;

mpu6050_t *mpu6050_init(w_i2c_config_t *i2c_params);
err_t mpu6050_setup(mpu6050_t sensor,imu_t *sensor_params);
err_t mpu6050_get_acce_raw(mpu6050_t *sensor,acce_raw *accel_data);
err_t mpu6050_get_gyro_raw(mpu6050_t *sensor,gyro_raw *gyro_data);
err_t mpu6050_get_acce_sensitivity(mpu6050_t *sensor,float *acce_sensitivity);
err_t mpu6050_get_gyro_sensitivity(mpu6050_t *sensor,float *gyro_sensitivity);
err_t mpu6050_set_acce_range(mpu6050_t *sensor, acel_range range);
err_t mpu6050_set_gyro_range(mpu6050_t *sensor, gyro_range range);
err_t mpu6050_get_acce_range(mpu6050_t *sensor, acel_range *range);
err_t mpu6050_get_gyro_range(mpu6050_t *sensor, gyro_range *range);
err_t mpu6050_get_gyro(mpu6050_t sensor, float *ax, float *ay, float *az);
err_t mpu6050_get_vel(mpu6050_t sensor, float *gx, float *gy, float *gz);
err_t mpu6050_get_orientation(mpu6050_t sensor, float *roll, float *pitch, float *yaw);

#endif