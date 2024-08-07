mpu6050_t *sensor mpu6050_init(i2c_bus_t *i2c_params);
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
Funciones privadas
err_t mpu6050_read_byte(mpu6050_t *sensor, uint8_t reg, uint8_t *data);
err_t mpu6050_write_byte(mpu6050_t *sensor, uint8_t reg_addr, uint8_t data);
err_t mpu6050_write(mpu6050_t *sensor, uint8_t start_addr, uint8_t num, uint8_t *buff);
err_t mpu6050_read(mpu6050_t *sensor, uint8_t start_addr, uint8_t num, uint8_t *buff);