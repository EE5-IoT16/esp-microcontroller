static const char *TAG = "main";
#include "i2c_sensor.h"
#include "pulse_counter.h"
#include <stdio.h>  
#include "nvs_flash.h"
#include "nvs.h"
#include "http_client.h"
#include "blufi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" 

nvs_handle_t data_handle;
esp_err_t err;

struct controller
{
    int sensorC;
    int counterC;
    int http_i2c;
    int http_bpm;
};

struct sensordata
{
    uint32_t steps;
    float temperature;
    int detectFall;
};

volatile struct controller controller;
struct sensordata sensordata;
int bpm;

void tSensor(void* arg)
{
    i2cSensor_init();
    do
    {
        readDataFromSensor(1000);
        err = nvs_get_u32(data_handle, "step_counter", &(sensordata.steps));
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
        err = nvs_get_u8(data_handle, "bpm", &bpm);


    }while (controller.counterC);
    unitializePulseCounter();
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
        vTaskDelay(5000/ portTICK_PERIOD_MS);
        if (wifi_connected())
        {
           sample_api_req_hardcoded(1,sensordata.steps,STEP);
        }
        else
        {
            char message[100]= " ";
            sprintf(message,"WIFI disconnted, your current step is %d ",sensordata.steps);
            uint32_t length = strlen(message);
            esp_blufi_send_custom_data((unsigned char*)message,length);
        }

    } while (controller.http_i2c);
    vTaskDelete(NULL);
}

void tHttpBpm(void* arg)
{
    do
    {   vTaskDelay(5000/ portTICK_PERIOD_MS);
        if (wifi_connected())
        {
            sample_api_req_hardcoded(1,bpm,HEARTRATE);
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
    while(1){
    ESP_LOGI(TAG, "main process starts to run\n");
    xTaskCreatePinnedToCore(tBlufi,"blufi",5000,NULL,2,NULL,tskNO_AFFINITY);
    while (!ble_connected())
    {
         vTaskDelay(500/ portTICK_PERIOD_MS);
    }
    //////////////////////////////////////////////////////////////////////////////////////
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... \n");
    err = nvs_open("storage", NVS_READWRITE, &data_handle);
    if (err != ESP_OK) 
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err)); 
    }
    err = nvs_open("storage", NVS_READONLY, &data_handle);
    //////////////////////////////////////////////////////////////////////////////////////

    controller.sensorC  = 1;
    controller.counterC = 1;
    controller.http_i2c = 1;
    controller.http_bpm = 1;
    xTaskCreatePinnedToCore(tSensor,"sensor",2048,NULL,1,NULL,tskNO_AFFINITY);
    xTaskCreatePinnedToCore(tPulse,"pulse",2048,NULL,1,NULL,tskNO_AFFINITY);
    vTaskDelay(500/ portTICK_PERIOD_MS);
    xTaskCreatePinnedToCore(tHttpSensor,"httpSensor",10240,NULL,1,NULL,tskNO_AFFINITY);
    xTaskCreatePinnedToCore(tHttpBpm,"httpBpm",10240,NULL,1,NULL,tskNO_AFFINITY);
    
    while(ble_connected())
    {
         vTaskDelay(500/ portTICK_PERIOD_MS);
    }
    controller.sensorC = 0;
    controller.counterC = 0;
    controller.http_i2c = 0;
    controller.http_bpm = 0;
    nvs_close(data_handle);
    ESP_LOGI(TAG, "device disconnected\n");
    }
}
