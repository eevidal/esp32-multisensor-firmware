#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <esp32/rom/ets_sys.h>
#include "time_module.h"
#include "i2c_module.h"

#define IMU 
#ifdef ULTRASONIC
#include "hcrs04.h"
#define ECHO_PIN GPIO_NUM_4
#define TRIGGER_PIN GPIO_NUM_5
#endif

#ifdef OPTICS
#include "apds9960.h"
#endif

#ifdef IMU
#include "mpu6050.h"
#endif



#ifdef OPTICS
TaskHandle_t gesture_task_handle = NULL;
apds9960_t sensor_a = NULL;
static const char *dtaga = "Gesture";
static void gesture_task(void *sens);
#endif
#ifdef IMU
TaskHandle_t imu_task_handle = NULL;
mpu6050_t sensor_m = NULL;
static const char *dtagm = "IMU";
static void imu_task(void *sens);
#endif
#ifdef ULTRASONIC
TaskHandle_t ultrasonic_task_handle = NULL;
hcrs04_t sensor_u = NULL;
static const char *dtagu = "Ultrasonic";
static void ultrasonic_task(void *sens);
#endif


static const char *dtag = "Main";

i2c_bus_t *i2c_bus = NULL;
w_i2c_config_t i2c_params = {
    .sda_num = GPIO_NUM_21,
    .scl_num = 22,       // GPIO_NUM_22,
    .clk_speed = 400000, // 100KHz
};


void app_main(void)
{
    #ifdef ULTRASONIC
    sensor_u = hcrs04_create(TRIGGER_PIN, ECHO_PIN, 6000);
    #endif

    #ifdef OPTICS
    i2c_bus  = i2c_init_bus(&i2c_params);

    sensor_a = apds9960_init(i2c_bus);
    #endif

    #ifdef IMU
				i2c_bus  = i2c_init_bus(&i2c_params);
    sensor_m = mpu6050_init(i2c_bus);
    #endif

    ESP_LOGI(dtag, "Creando Tareas\n");
    #ifdef ULTRASONIC
				xTaskCreate(&ultrasonic_task, "Ultrasonic", 2048, (void *)sensor_u, 5, &ultrasonic_task_handle);
    #endif
				#ifdef IMU
    xTaskCreate(&imu_task, "Position", 2048, (void *)sensor_m, 5, &imu_task_handle);
    #endif
    #ifdef OPTICS
    xTaskCreate(&gesture_task, "Gesture", 2048, (void *)sensor_a, 6, &gesture_task_handle);
    #endif

  //  configASSERT(ultrasonic_task_handle);
 //   configASSERT(gesture_task_handle);
 //   configASSERT(imu_task_handle);

    while (1)
        vTaskDelay(1000);

    #ifdef ULTRASONIC
    if (ultrasonic_task_handle != NULL)
    {
        ESP_LOGI(dtagu, "DELETING...");
        vTaskDelete(ultrasonic_task_handle);
    }
				#endif
				#ifdef OPTICS
    if (gesture_task_handle != NULL)
    {
        ESP_LOGI(dtaga, "DELETING...");
        vTaskDelete(gesture_task_handle);
    }
				#endif
				#ifdef IMU
    if (imu_task_handle != NULL)
    {
        ESP_LOGI(dtagm, "DELETING...");
        vTaskDelete(imu_task_handle);
    }
				#endif
}
/* **********************************************************/

#ifdef ULTRASONIC
static void ultrasonic_task(void *sens)
{
    hcrs04_t *sensor = (hcrs04_t *)sens;
    while (1)
    {
        float distance;
        ESP_LOGD(dtagu, "Obteniendo Distancia\n");
        hcrs04_get_distance_m(sensor, &distance);
        ESP_LOGI(dtagu, "Distancia %0.5fcm\n", distance * 100);
        ESP_LOGD(dtagu, "tigger %d", hcrs04_echo_pin(sensor));
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}
#endif

#ifdef OPTICS
static void gesture_task(void *sens)
{
    apds9960_t *sensor = (apds9960_t *)sens;
    ESP_LOGI(dtaga, "Iniciando Engine de Gestos\n");
    uint8_t val = 0;
    apds9960_get_id(sensor, &val);
    printf("ID apds9960 %X \n", val);
    apds9960_gesture_init(sensor);
  //  apds9960_diagnose(sensor);
    apds9960_set_timeout(sensor, 40);
    uint8_t gesture = 0;
    while (1)
    {
        ESP_LOGD(dtaga, "Obteniendo Gesto\n");
        apds9960_read_gesture(sensor, &gesture);
        if (gesture == DOWN)
        {
            printf("DOWN!\n");
        }
        else if (gesture == UP)
        {
            printf("UP!\n");
        }
        else if (gesture == LEFT)
        {
            printf("LEFT!\n");
        }
        else if (gesture == RIGHT)
        {
            printf("RIGHT!\n");
        }
        else if (gesture == FAR)
        {
            printf("FAR!\n");
        }
        vTaskDelay(pdMS_TO_TICKS(600));
    }
};

#endif

#ifdef IMU
static void imu_task(void *sens)
{
    mpu6050_t *sensor = (mpu6050_t *)sens;
    ESP_LOGD(dtagm, "Iniciando Imu\n");
    uint8_t val = 0;
    mpu6050_get_id(sensor, &val);
    printf("ID mpu6050 %X \n", val);
    mpu6050_setup_default(sensor);

    acce_raw_t *acce = malloc(sizeof(acce_raw_t));
    gyro_raw_t *gyro = malloc(sizeof(gyro_raw_t));
    mpu6050_acce_t accel;
    mpu6050_gyro_t gyros;
    while (true)
    {
        mpu6050_get_gyro_raw(sensor, gyro);
        mpu6050_get_acce_raw(sensor, acce);
        printf("acce x:%X, y:%X, z:%X\n", acce->x, acce->y, acce->z);
       
        printf("gyro x:%X, y:%X, z:%X\n", gyro->x, gyro->y, gyro->z);
								mpu6050_get_gyro(sensor, &gyros);
        mpu6050_get_acce(sensor, &accel); 
        ESP_LOGI(dtagm, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", accel.x, accel.y, accel.z);
        
        ESP_LOGI(dtagm, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyros.x, gyros.y, gyros.z);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}
#endif