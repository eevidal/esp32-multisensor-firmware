#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_log.h"

#include "time_module.h"
#include "hcrs04.h"
#include "apds9960.h"
#include "mpu6050.h"

//#define CONFIG_FREERTOS_HZ=100 

#define ECHO_PIN 4
#define TRIGGER_PIN 5
//TaskHandle_t gesture_task_handle = NULL;
//TaskHandle_t position_task_handle = NULL;

static const char* dtag = "Ultrasonic";

//apds9960_t* sensor2;
//mpu6050_t* sensor3;

//i2c
//w_i2c_config_t*  i2c_params; 

  //  i2c_params->sda_num = 26;
   // .scl_num = 29,
  //  .clk_speed = 400000, //40Kz#include "esp_system.h"

 hcrs04_t sensor =NULL;

static void ultrasonic_task(void* sens){
    float distance;
    sensor = hcrs04_create(TRIGGER_PIN, ECHO_PIN, 1000);
    while (1)
    {
        ESP_LOGI(dtag, "Obteniendo Distancia\n");
        distance = hcrs04_get_distance_m(sensor);
        ESP_LOGI(dtag, "Distancia %f", distance);
       
        delay(500);   
      
    }
}

/* static void task1(void* param){
    while(1){
        printf("en la tarea\n");
        delay(10);

    }
} */
/* static void gesture_task(apds9960_t * sens){
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
}; */
    

/* static void position_task(mpu6050_t * sens){
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
} */


void app_main(void)
{
   // TaskHandle_t distance_task_handle = NULL;
   // TaskHandle_t handler = NULL;
   
    vTaskDelay(100);
     //echo 20 trigger 23
 //   i2c_params->sda_num = 21;
 //   i2c_params->scl_num= 22;
 //   i2c_params->clk_speed = 400000;
 //   sensor2 = apds9960_init(i2c_params);
 //   sensor3 = mpu6050_init(i2c_params);
    printf("hola\n");
     xTaskCreate(&ultrasonic_task, "Ultrasonic", 1024, NULL, 4, NULL);
  //   xTaskCreate(&task1,"task", 1024, NULL, 5, &handler);

}
