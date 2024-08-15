#ifndef _MPU_6050_H_
#define _MPU_6050_H_

#include "error_module.h"
#include "i2c_module.h"

typedef void* mpu6050_t;

typedef enum{
    ACCE_2G  = 0,     /*!< Accelerometer full scale range is +/- 2g */
    ACCE_4G  = 1,     /*!< Accelerometer full scale range is +/- 4g */
    ACCE_8G  = 2,     /*!< Accelerometer full scale range is +/- 8g */
    ACCE_16G = 3,     /*!< Accelerometer full scale range is +/- 16g */
} acel_range;

typedef enum{
    GYRO_250DPS  = 0,     /*!< Gyroscope full scale range is +/- 250 degree per second */
    GYRO_500DPS  = 1,     /*!< Gyroscope full scale range is +/- 500 degree per second */
    GYRO_1000DPS = 2,     /*!< Gyroscope full scale range is +/- 1000 degree per second */
    GYRO_2000DPS = 3,     /*!< Gyroscope full scale range is +/- 2000 degree per second */

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

mpu6050_t *mpu6050_init(i2c_bus_t* i2c_bus);
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