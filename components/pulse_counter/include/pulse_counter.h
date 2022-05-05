#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_blufi.h"

typedef enum
{
    NORMAL,
    HIGH,
    WARNING
} HeartRateStatus;

void counter_init(void);
HeartRateStatus startToCount(int period);
void counter_init(void);
void unitializePulseCounter(void);