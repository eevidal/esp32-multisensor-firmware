#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "time_module.h"
#include "hcrs04.h"
#include "apds9960.h"
//#include "mpu6050.h"


#define ECHO_PIN GPIO_NUM_4
#define TRIGGER_PIN GPIO_NUM_5

TaskHandle_t gesture_task_handle = NULL;
//TaskHandle_t imu_task_handle = NULL;
TaskHandle_t ultrasonic_task_handle = NULL; 
hcrs04_t sensor_u = NULL;
apds9960_t sensor_a = NULL;
//mpu6050_t sensor_m = NULL;
i2c_bus_t* i2c_bus = NULL;


static const char* dtagu = "Ultrasonic";
static const char* dtaga = "Gesture";
static const char* dtag = "Main";
//static const char* dtagm = "IMU";

static void ultrasonic_task(void* sens);
static void gesture_task(void * sens);
//static void imu_task(void * sens);

//i2c
w_i2c_config_t i2c_params = {
    .sda_num = GPIO_NUM_21,
    .scl_num = GPIO_NUM_22,
    .clk_speed = 400000, //40Kz
}; 



void app_main(void)
{
    
    sensor_u = hcrs04_create(TRIGGER_PIN, ECHO_PIN, 6000);
 
    i2c_bus =  i2c_init_bus(&i2c_params);
    sensor_a = apds9960_init(i2c_bus);

 //   sensor_m = mpu6050_init(i2c_bus);
    ESP_LOGI(dtag, "Creando Tareas\n");
    xTaskCreate( &ultrasonic_task, "Ultrasonic",2048,(void *)sensor_u, 5, &ultrasonic_task_handle);
    xTaskCreate( &gesture_task, "Gesture",2048,(void *)sensor_a, 10, &gesture_task_handle);
   // xTaskCreate( &imu_task, "Position",2048,(void *)sensor_m, 5, &imu_task_handle);

    configASSERT(ultrasonic_task_handle);
    configASSERT(gesture_task_handle);
   // configASSERT(imu_task_handle);

   while(1) vTaskDelay(1000);
   
   if( ultrasonic_task_handle != NULL )
    {
        ESP_LOGI(dtagu, "DELETING...");
        vTaskDelete(ultrasonic_task_handle );
    }
   if( gesture_task_handle != NULL )
    {
        ESP_LOGI(dtaga, "DELETING...");
        vTaskDelete(gesture_task_handle );
    }
   /*     if(imu_task_handle != NULL )
    {
        ESP_LOGI(dtagm, "DELETING...");
        vTaskDelete(imu_task_handle );
    } */
}
/* **********************************************************/


 static void ultrasonic_task(void* sens)
 {
   hcrs04_t * sensor = (hcrs04_t * )sens;
 //   static portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
    while (1)
    {
        float distance;
        ESP_LOGI(dtagu, "Obteniendo Distancia\n");
      //  portENTER_CRITICAL_SAFE(&mutex);
        hcrs04_get_distance_m(sensor, &distance);
   //     portEXIT_CRITICAL_SAFE(&mutex);
        ESP_LOGI(dtagu, "Distancia %0.5fcm\n", distance*100);
        ESP_LOGD(dtagu, "tigger %d", hcrs04_echo_pin(sensor));
       vTaskDelay(pdMS_TO_TICKS(1500));    
    }
} 

 static void gesture_task(void * sens){
    apds9960_t * sensor = (apds9960_t *)sens;
    ESP_LOGI(dtaga, "Iniciando Engine de Gestos\n");
    apds9960_gesture_init(sensor);
    apds9960_gesture_t gesture;
    while (true) {
        ESP_LOGD(dtaga, "Obteniendo Gesto\n");
         //  portENTER_CRITICAL_SAFE(&mutex);
        apds9960_read_gesture(sensor, &gesture);
         //     portEXIT_CRITICAL_SAFE(&mutex);
        if (gesture == DOWN) {
            printf("DOWN!\n");
        } else if (gesture == UP) {
            printf("UP!\n");
        } else if (gesture == LEFT) {
            printf("LEFT!\n");
        } else if (gesture == RIGHT) {
            printf("RIGHT!\n"); 
        } 
        vTaskDelay(pdMS_TO_TICKS(200)); 
    }
};     

/*  static void imu_task(void * sens){
{
    mpu6050_t * sensor = (mpu6050_t *)sens;
    mpu6050_acce_t acce;
    mpu6050_gyro_t gyro;
    mpu6050_wake_up(sensor);
    mpu6050_set_acce_fs(sensor, ACCE_FS_4G);
    mpu6050_set_gyro_fs(sensor, GYRO_FS_500DPS);

    while (true) { 
        mpu6050_get_acce(sensor, &acce);
        printf("acce x:%.2f, y:%.2f, z:%.2f\n", acce.x, acce.y, acce.z);
        mpu6050_get_gyro(sensor, &gyro);
        printf("gyro x:%.2f, y:%.2f, z:%.2f\n", gyro.x, gyro.y, gyro.z);
        vTaskDelay(pdMS_TO_TICKS(300)); 
    }
} 
 */