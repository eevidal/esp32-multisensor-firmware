#ifndef _MPU_6050_H_
#define _MPU_6050_H_

#include "error_module.h"
#include "time_module.h"
#include "i2c_module.h"

typedef void* mpu6050_t;

typedef enum{ 
    CLK_INTERNAL    =  0x00, 
    CLK_PLL_XGYRO   =  0x01,
    CLK_PLL_YGYRO    = 0x02,
    CLK_PLL_ZGYRO    = 0x03,
    CLK_PLL_EXT32K   = 0x04,
    CLK_PLL_EXT19M   = 0x05,
    CLK_KEEP_RESET   = 0x07,
} mpu6050_pwr_clk_t;

typedef enum{
    ACCE_2G  = 0x00,     /*!< Accelerometer full scale range is +/- 2g */
    ACCE_4G  = 0x08,     /*!< Accelerometer full scale range is +/- 4g */
    ACCE_8G  = 0x10,     /*!< Accelerometer full scale range is +/- 8g */
    ACCE_16G = 0x18,     /*!< Accelerometer full scale range is +/- 16g */
} acel_range_t;

typedef enum{
    GYRO_250DPS  = 0x00,     /*!< Gyroscope full scale range is +/- 250 degree per second */
    GYRO_500DPS  = 0x08,     /*!< Gyroscope full scale range is +/- 500 degree per second */
    GYRO_1000DPS = 0x10,     /*!< Gyroscope full scale range is +/- 1000 degree per second */
    GYRO_2000DPS = 0x18,     /*!< Gyroscope full scale range is +/- 2000 degree per second */

} gyro_range_t;

typedef enum {
    DLPF_256HZ       = 0, /*Accel 260*/
    DLPF_188HZ       = 1, /*Accel 184*/
    DLPF_98HZ        = 2, /*Accel 94*/
    DLPF_42HZ        = 3, /*Accel 44*/
    DLPF_20HZ        = 4, /*Accel 21*/
    DLPF_10HZ        = 5, /*Accel10*/
    DLPF_5HZ         = 6, /*Accel5*/
} dlpf_t;

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

mpu6050_t *mpu6050_create(i2c_bus_t* i2c_bus);
err_t mpu6050_delete(mpu6050_t *sensor);
err_t mpu6050_get_id(mpu6050_t *sensor, uint8_t *id);
err_t mpu6050_init(mpu6050_t* sensor);// mov to aplication layer
err_t mpu6050_set_pwr_clock(mpu6050_t *sensor, mpu6050_pwr_clk_t mode);
err_t mpu6050_set_dlpf(mpu6050_t *sensor, dlpf_t filter);
err_t mpu6050_set_sample_rate(mpu6050_t *sensor, int16_t rate);
err_t mpu6050_set_acce_range(mpu6050_t *sensor, acel_range_t range);
err_t mpu6050_set_gyro_range(mpu6050_t *sensor,  gyro_range_t range);
err_t mpu6050_get_acce_raw(mpu6050_t *sensor,acce_raw_t *accel_data);
err_t mpu6050_get_gyro_raw(mpu6050_t *sensor,gyro_raw_t *gyro_data);
err_t mpu6050_get_acce_sensitivity(mpu6050_t *sensor,float *acce_sensitivity);
err_t mpu6050_get_gyro_sensitivity(mpu6050_t *sensor,float *gyro_sensitivity);
err_t mpu6050_get_acce_range(mpu6050_t *sensor, acel_range_t *range);
err_t mpu6050_get_gyro_range(mpu6050_t *sensor,  gyro_range_t *range);
err_t mpu6050_get_gyro(mpu6050_t * sensor, mpu6050_gyro_t *gyros);
err_t mpu6050_get_acce(mpu6050_t * sensor, mpu6050_acce_t *accel);
err_t mpu6050_get_orientation(mpu6050_t sensor, float *roll, float *pitch, float *yaw);

#endif