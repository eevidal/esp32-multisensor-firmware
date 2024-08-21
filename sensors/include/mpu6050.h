#ifndef _MPU_6050_H_
#define _MPU_6050_H_

#include "error_module.h"
#include "i2c_module.h"

typedef void* mpu6050_t;

typedef enum{ 
    CLK_INTERNAL_C    =  0x20, //Cicle
    CLK_PLL_XGYRO_C   =  0x21,
    CLK_PLL_YGYRO_C    = 0x22,
    CLK_PLL_ZGYRO_C    = 0x23,
    CLK_PLL_EXT32K_C   = 0x24,
    CLK_PLL_EXT19M_C   = 0x25,
    CLK_KEEP_RESET_C   = 0x27,
    CLK_INTERNAL_S     = 0x40, //Sleep
    CLK_PLL_XGYRO_S    = 0x41,
    CLK_PLL_YGYRO_S    = 0x42,
    CLK_PLL_ZGYRO_S    = 0x43,
    CLK_PLL_EXT32K_S   = 0x44,
    CLK_PLL_EXT19M_S   = 0x45,
    CLK_KEEP_RESET_S   = 0x47
} mpu6050_pwr_clk_t;

typedef enum{
    ACCE_2G  = 0x00,     /*!< Accelerometer full scale range is +/- 2g */
    ACCE_4G  = 0x08,     /*!< Accelerometer full scale range is +/- 4g */
    ACCE_8G  = 0x10,     /*!< Accelerometer full scale range is +/- 8g */
    ACCE_16G = 0x18,     /*!< Accelerometer full scale range is +/- 16g */
} acel_range_t;

typedef enum{
    GYRO_250DPS  = 0,     /*!< Gyroscope full scale range is +/- 250 degree per second */
    GYRO_500DPS  = 1,     /*!< Gyroscope full scale range is +/- 500 degree per second */
    GYRO_1000DPS = 2,     /*!< Gyroscope full scale range is +/- 1000 degree per second */
    GYRO_2000DPS = 3,     /*!< Gyroscope full scale range is +/- 2000 degree per second */

} gyro_range_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;

} acce_raw_t;

typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;
    
} gyro_raw_t;

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
    mpu6050_pwr_clk_t mode;


} imu_t;

typedef void * mpu6050_t;

mpu6050_t *mpu6050_init(i2c_bus_t* i2c_bus);
err_t mpu6050_delete(mpu6050_t *sensor);
err_t mpu6050_get_id(mpu6050_t *sensor);
err_t mpu6050_setup_default(mpu6050_t* sensor);// mov to aplication layer
err_t mpu6050_set_pwr_clock(mpu6050_t *sensor, mpu6050_pwr_clk_t mode);
err_t mpu6050_get_acce_raw(mpu6050_t *sensor,acce_raw_t *accel_data);
err_t mpu6050_get_gyro_raw(mpu6050_t *sensor,gyro_raw_t *gyro_data);
err_t mpu6050_get_acce_sensitivity(mpu6050_t *sensor,float *acce_sensitivity);
err_t mpu6050_get_gyro_sensitivity(mpu6050_t *sensor,float *gyro_sensitivity);
err_t mpu6050_set_acce_range(mpu6050_t *sensor, acel_range_t range);
err_t mpu6050_set_gyro_range(mpu6050_t *sensor,  gyro_range_t range);
err_t mpu6050_get_acce_range(mpu6050_t *sensor, acel_range_t *range);
err_t mpu6050_get_gyro_range(mpu6050_t *sensor,  gyro_range_t *range);
err_t mpu6050_get_gyro(mpu6050_t sensor, float *ax, float *ay, float *az);
err_t mpu6050_get_vel(mpu6050_t sensor, float *gx, float *gy, float *gz);
err_t mpu6050_get_orientation(mpu6050_t sensor, float *roll, float *pitch, float *yaw);

#endif