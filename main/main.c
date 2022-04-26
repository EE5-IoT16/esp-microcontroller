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
    int http_i2c;
    int http_bpm;
};

struct sensordata
{
    int steps;
};

volatile struct controller controller;
//volatile sensordata sensordata;
volatile int bpm;

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
        bpm = getHeartRate(); 

    }while (controller.counterC);
    vTaskDelete(NULL);

}
void tBlufi(void* arg)
{
    run_blufi();
    vTaskDelete(NULL);
}

void  tHttpSensor(void* arg)
{
    do
    {
        if (wifi_connected())
        {
            
        }
        
    } while (controller.http_i2c);
    vTaskDelete(NULL);
}

void tHttpBpm(void* arg)
{
    do
    {   vTaskDelay(10000/ portTICK_PERIOD_MS);
        if (wifi_connected())
        {
            sample_api_req_hardcoded(1,bpm);
        }
        else 
        {
            char message[100]= " ";
            sprintf(message,"WIFI disconnted, your current heartrate is %d bpm",bpm);
            uint32_t length = strlen(message);
            esp_blufi_send_custom_data((unsigned char*)message,length);
        }
        
    } while (controller.http_bpm);
     vTaskDelete(NULL);
}

void app_main(void)
{
    
    ESP_LOGI(TAG, "main process starts to run\n");
    xTaskCreatePinnedToCore(tBlufi,"blufi",4096,NULL,2,NULL,tskNO_AFFINITY);
    
    while (!ble_connected())
    {
         vTaskDelay(500/ portTICK_PERIOD_MS);
    }
    controller.sensorC = 1;
    controller.counterC = 1;
    controller.http_i2c = 1;
    controller.http_bpm = 1;
    xTaskCreatePinnedToCore(tSensor,"sensor",2048,NULL,1,NULL,tskNO_AFFINITY);
    xTaskCreatePinnedToCore(tPulse,"pulse",2048,NULL,1,NULL,tskNO_AFFINITY);
    xTaskCreatePinnedToCore(tHttpSensor,"httpSensor",2048,NULL,1,NULL,tskNO_AFFINITY);
    xTaskCreatePinnedToCore(tHttpBpm,"httpBpm",2048,NULL,1,NULL,tskNO_AFFINITY);
    while(ble_connected())
    {
         vTaskDelay(5000/ portTICK_PERIOD_MS);
    }
    controller.sensorC = 0;
    controller.counterC = 0;
    controller.http_i2c = 0;
    controller.http_bpm = 0;
    ESP_LOGI(TAG, "device disconnected\n");
}
