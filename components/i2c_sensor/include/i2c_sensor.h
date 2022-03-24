#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"


void getAccelOffset(void);
void getGyroOffset(void);
void stepCounter(short ay);
void i2cSensor_init(void);
void readDataFromSensor(int frequency);
void unitializedI2C(void);