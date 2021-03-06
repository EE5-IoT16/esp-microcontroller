#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "esp_blufi.h"
void getAccelOffset(void);
void getGyroOffset(void);
void stepCounter(short ay);
void i2cSensor_init(void);
void step_counter(void);
void unitializedI2C(void);
void measure_temperature(void);
int detect_fall(void);