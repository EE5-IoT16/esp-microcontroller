static const char *TAG = "iot_16_main";
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
HeartRateStatus status;

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
    short temperature;
    int16_t fall;
};

volatile struct controller controller;
static struct sensordata sensordata;
static u_int16_t bpm;


void tStep(void* arg)
{
    i2cSensor_init();
    do
    {
        vTaskDelay(50/portTICK_PERIOD_MS);
        step_counter();
    }
    while(controller.sensorC);
    unitializedI2C();
    vTaskDelete(NULL);
   
}

void tTemperature(void* arg)
{
    do
    {
        vTaskDelay(2000/portTICK_PERIOD_MS);
        measure_temperature();
    }
    while(controller.sensorC); 
    vTaskDelete(NULL);
}

void tFallDetect(void* arg)
{   
    do
    {
        vTaskDelay(25/portTICK_PERIOD_MS);
        sensordata.fall = detect_fall();
        if (sensordata.fall != 0)
        {
            if (wifi_connected())
            {
                sample_api_req_hardcoded(1,sensordata.fall,FALLS);
            }
            else 
            {
                char message[100]= " ";
                sprintf(message,"Detect a fall, possibility: %d percent",sensordata.fall*33);
                uint32_t length = strlen(message);
                esp_blufi_send_custom_data((unsigned char*)message,length);
            }
            vTaskDelay(1000/portTICK_PERIOD_MS);
        }  
    }
    while(controller.sensorC); 
    vTaskDelete(NULL);
}

void tPulse(void* arg)
{
    counter_init();
    do
    {
        status = startToCount(5000);
        if(nvs_get_u16(data_handle, "bpm", &bpm) != ESP_OK)
        {
            char message[100]= " ";
            sprintf(message,"Reading heartrate data error %d ",sensordata.steps);
            uint32_t length = strlen(message);
            esp_blufi_send_custom_data((unsigned char*)message,length);
        }
        

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
        if(nvs_get_u32(data_handle, "step_counter", &(sensordata.steps)) != ESP_OK)
        {
            char message[100]= " ";
            sprintf(message,"Reading steps data error ");
            uint32_t length = strlen(message);
            esp_blufi_send_custom_data((unsigned char*)message,length);
        }
        if(nvs_get_i16(data_handle, "temperature", &(sensordata.temperature)) != ESP_OK)
        {
            char message[100]= " ";
            sprintf(message,"Reading temperature data error");
            uint32_t length = strlen(message);
            esp_blufi_send_custom_data((unsigned char*)message,length);
        }
        if (wifi_connected())
        {
            sample_api_req_hardcoded(1,sensordata.steps,STEPS);
            sample_api_req_hardcoded(1,sensordata.temperature,TEMPERATURE);
        }
        else
        {
            char message[100]= " ";
            sprintf(message,"WIFI disconnted, your current step is %d ",sensordata.steps);
            uint32_t length = strlen(message);
            esp_blufi_send_custom_data((unsigned char*)message,length);

            float  temperature = 21.00f + ((((float)(sensordata.temperature)-333.87f*6))/333.87f);
            sprintf(message,"WIFI disconnted, your current temperature is %.2f ",temperature);
            length = strlen(message);
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
            switch (status)
            {
            case NORMAL:
                sprintf(message,"WIFI disconnted, your current heartrate is %d bps, NORMAL",bpm);
                break;
            case HIGH:
                sprintf(message,"WIFI disconnted, your current heartrate is %d bps, HIGH",bpm);
                break;
             case WARNING:
                sprintf(message,"WIFI disconnted, your current heartrate is %d bps, WARNING",bpm);
                break;    
            default:
                break;
            }
           
            uint32_t length = strlen(message);
            esp_blufi_send_custom_data((unsigned char*)message,length);
        }
    } while (controller.http_bpm);
     vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t err;
    while(1){
    ESP_LOGI(TAG, "main process starts to run\n");
    char message[100]= " ";
    xTaskCreatePinnedToCore(tBlufi,"blufi",4096,NULL,2,NULL,tskNO_AFFINITY);
    while (!ble_connected())
    {
         vTaskDelay(500/ portTICK_PERIOD_MS);
         
    }
    vTaskDelay(2000/ portTICK_PERIOD_MS);

    //////////////////////////////////////////////////////////////////////////////////////
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        err = nvs_flash_erase();
        err = nvs_flash_init();
         if (err != ESP_OK) 
         {
            sprintf(message,"Error (%s) erase NVS handle!\n", esp_err_to_name(err));
            uint32_t length = strlen(message);
            esp_blufi_send_custom_data((unsigned char*)message,length);
         }
    }
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... \n");
    err = nvs_open("storage", NVS_READWRITE, &data_handle);
    if (err != ESP_OK) 
    {
        sprintf(message,"Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        uint32_t length = strlen(message);
        esp_blufi_send_custom_data((unsigned char*)message,length);
    }
    err = nvs_open("storage", NVS_READONLY, &data_handle);
    ///////////////////////////////////////////////////////////////////////////////////////
    controller.sensorC  = 1;
    controller.counterC = 1;
    controller.http_i2c = 1;
    controller.http_bpm = 1;
    xTaskCreatePinnedToCore(tStep,"step",4096,NULL,1,NULL,tskNO_AFFINITY);
    vTaskDelay(5000/ portTICK_PERIOD_MS);
    //xTaskCreatePinnedToCore(tTemperature,"temperature",4096,NULL,1,NULL,tskNO_AFFINITY);
    xTaskCreatePinnedToCore(tFallDetect,"fall",4096,NULL,1,NULL,tskNO_AFFINITY);
    //xTaskCreatePinnedToCore(tPulse,"pulse",4096,NULL,1,NULL,tskNO_AFFINITY);
    vTaskDelay(5000/ portTICK_PERIOD_MS);
    //xTaskCreatePinnedToCore(tHttpSensor,"httpSensor",10240,NULL,1,NULL,tskNO_AFFINITY);
    //xTaskCreatePinnedToCore(tHttpBpm,"httpBpm",10240,NULL,1,NULL,tskNO_AFFINITY);
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
