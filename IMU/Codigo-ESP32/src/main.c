#include "stdio.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "mpu6050_wrapper.h"

#define PIN_LED 27

// Pinout MPU6050
#define GPIO_MPU_INT        31      
#define GPIO_MPU_SDA        32
#define GPIO_MPU_SCL        33
#define MPU_HANDLER_PRIORITY    5//configMAX_PRIORITIES - 1
#define IMU_HANDLER_CORE        APP_CPU_NUM     // core 1

extern QueueHandle_t mpu6050QueueHandler; 

static void runningLed(void *pvParameters);
static void showAngles(void *pvParameters);

void app_main(){

    gpio_set_direction(PIN_LED,GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED,1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(PIN_LED,0);
    
    mpu6050QueueHandler = xQueueCreate(1,sizeof(vector_queue_t));

    mpu6050_init_t configMpu = {
        .intGpio = GPIO_MPU_INT,
        .sclGpio = GPIO_MPU_SCL,
        .sdaGpio = GPIO_MPU_SDA,
        .priorityTask = MPU_HANDLER_PRIORITY,
        .core = IMU_HANDLER_CORE
    };
    mpu6050_initialize(&configMpu);

    //xTaskCreate(runningLed,"led running",2048,NULL,3,NULL);
    xTaskCreate(showAngles,"show angles",2048,NULL,4,NULL);

}


static void showAngles(void *pvParameters){
    // rawData_t rawData;

    vector_queue_t newAngles;

    while(1){
        // printf("Angulo X: %f  angulo Y: %f  distancia: %d cms\n",getAngle(AXIS_ANGLE_X),getAngle(AXIS_ANGLE_Y),tfMiniGetDist());
        
        // rawData = getRawData();

        // printf("%d,%d,%d,%d,%d,%d\n",rawData.accX,rawData.accY,rawData.accZ,rawData.gyX,rawData.gyY,rawData.gyZ);
        // vTaskDelay(pdMS_TO_TICKS(2));

        if(xQueueReceive(mpu6050QueueHandler,&newAngles,pdMS_TO_TICKS(10))) {

            printf("X: %f,Y: %f,Z: %f\r\n",newAngles.pitch,newAngles.roll,newAngles.yaw);
        }
    }
}

static void runningLed(void *pvParameters){


    while(1){
        gpio_set_level(PIN_LED,0);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(PIN_LED,1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}