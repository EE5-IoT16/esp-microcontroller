static const char *TAG = "main";

#include "i2c_sensor.h"
#include "pulse_counter.h"
#include <stdio.h>  
#include "http_client.h"
#include "blufi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" 

struct controller
{
    int sensorC;
    int counterC;
};

volatile struct controller controller;

void tSensor(void* arg)
{
    i2cSensor_init();
    do
    {
        readDataFromSensor(1000);
    }
    while(controller.sensorC);
    unitializedI2C();
    vTaskDelete(NULL);
   
}


void tPulse(void* arg)
{
    counter_init();
    do
    {
        startToCount(5000);

    }while (controller.counterC);
    vTaskDelete(NULL);

}


void app_main(void)
{
    
    ESP_LOGI(TAG, "main process starts to run");
    /*
    controller.sensorC = 1;
    controller.counterC = 1;
    xTaskCreatePinnedToCore(tSensor,"sensor",2048,NULL,1,NULL,tskNO_AFFINITY);
    xTaskCreatePinnedToCore(tPulse,"pulse",2048,NULL,2,NULL,tskNO_AFFINITY);
    vTaskDelay(300000/ portTICK_PERIOD_MS);
    controller.sensorC = 0;
    controller.counterC = 0;
    */
    //sample_api_req_hardcoded(50, 12345);
    run_blufi();
}
