#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "hcrs04.h"
#include "apds9960.h"
#include "mpu6050.h"

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
        printf("Distancia medida %d", distance);
        vTaskDelay(pdMS_TO_TICKS(10));
        
    }
}

static void gesture_task(apds9960_t * sens){
     while (true)
    {

    }
    
};
static void position_task(mpu6050_t * sens){
    
};


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