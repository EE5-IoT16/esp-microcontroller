#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
//#include "protocol_examples_common.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"
#include <stdarg.h>
#include <stdio.h>
#include "esp_http_client.h"

typedef enum
{
    HEARTRATE,
    ACCEL,
    GYRO,
    TEMPERATURE,
    STEP
} API_NODES_t;

esp_err_t _http_event_handler(esp_http_client_event_t *evt);
void run(void);

void sample_api_req_hardcoded(int deviceID, int data, API_NODES_t type);