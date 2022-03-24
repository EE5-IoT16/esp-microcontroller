static const char *TAG = "main";

#include "i2c_sensor.h"
#include "pulse_counter.h"
#include <pthread.h>
#include <stdio.h>   

struct controller
{
    int sensorC;
    int counterC;
};
volatile struct controller controller;



void* tSensor(void* arg)
{
    i2cSensor_init();
    do
    {
        readDataFromSensor(1000);
    }
    while(controller.sensorC);
   
    unitializedI2C();
    pthread_exit(NULL);
}


void* tPulse(void* arg)
{
    counter_init();
    do
    {
        startToCount(5000);

    }while (controller.counterC);
    pthread_exit(NULL);

}


void app_main(void)
{
    ESP_LOGI(TAG, "main process starts to run");
    pthread_t sensor, pulseCounter;
    controller.sensorC = 1;
    controller.counterC = 1;
    pthread_create(&sensor, NULL, tSensor, NULL);
	pthread_create(&pulseCounter, NULL, tPulse, NULL);


    vTaskDelay(8000/ portTICK_PERIOD_MS);
    controller.sensorC = 0;
    controller.counterC = 0;

    pthread_join(sensor,NULL);
    pthread_join(pulseCounter,NULL);
    ESP_LOGI(TAG, "main process ends");
}


