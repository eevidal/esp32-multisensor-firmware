#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "hcrs04.h"
#include "apds9960.h"
#include "mpu6050.h"

#define CONFIG_FREERTOS_HZ=100 

TaskHandle_t distance_task_handle = NULL;
TaskHandle_t gesture_task_handle = NULL;
TaskHandle_t position_task_handle = NULL;


hcrs04_t* sensor1;
apds9960_t* sensor2;
mpu6050_t* sensor3;

//i2c
w_i2c_config_t*  i2c_params; 

  //  i2c_params->sda_num = 26;
   // .scl_num = 29,
  //  .clk_speed = 400000, //40Kz


static void ultrasonic_task(hcrs04_t* sens){
    float distance;
    while (true)
    {
        hcrs04_get_distance_m(sens, &distance);
        printf("Distance %d", distance);
        delay(10);      
    }
}

static void gesture_task(apds9960_t * sens){
    apds9960_gesture_t *gesture;
    while (true) {
        uint8_t gesture = apds9960_read_gesture(sens, &gesture);
        if (gesture == DOWN) {
            printf("DOWN!\n");
        } else if (gesture == UP) {
            printf("UP!\n");
        } else if (gesture == LEFT) {
            printf("LEFT!\n");
        } else if (gesture == RIGHT) {
            printf("RIGHT!\n"); 
        } 
        delay(100);
    }
};
    

static void position_task(mpu6050_t * sens){
{
    mpu6050_acce_t acce;
    mpu6050_gyro_t gyro;
    mpu6050_wake_up(sens);
    mpu6050_set_acce_fs(sens, ACCE_FS_4G);
    mpu6050_set_gyro_fs(sens, GYRO_FS_500DPS);

    while (true) { 
        mpu6050_get_acce(sens, &acce);
        printf("acce x:%.2f, y:%.2f, z:%.2f\n", acce.x, acce.y, acce.z);
        mpu6050_get_gyro(sens, &gyro);
        printf("gyro x:%.2f, y:%.2f, z:%.2f\n", gyro.x, gyro.y, gyro.z);
        delay(100);
    }
}


main()
{
    sensor1 = hcrs04_create(23, 20, 10000); //echo 20 trigger 23
    i2c_params->sda_num = 26;
    i2c_params->sda_num = 29;
    i2c_params->sda_num = 400000;
    sensor2 = apds9960_init(i2c_params);
 //   sensor3 = mpu6050_init(i2c_params);

    xTaskCreate(&ultrasonic_task, "Ultrasonic", 512, NULL, 2, &distance_task_handle);

};


//scl 29
// sda 26