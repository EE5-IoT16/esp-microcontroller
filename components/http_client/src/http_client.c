/* ESP HTTP Client Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "http_client.h"

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048
static const char *TAG = "HTTP_CLIENT";

/* Root cert for howsmyssl.com, taken from howsmyssl_com_root_cert.pem

   The PEM file was extracted from the output of this command:
   openssl s_client -showcerts -connect www.howsmyssl.com:443 </dev/null

   The CA root cert is the last cert given in the chain of certs.

   To embed it in the app binary, the PEM file is named
   in the component.mk COMPONENT_EMBED_TXTFILES variable.
*/
// extern const char howsmyssl_com_root_cert_pem_start[] asm("_binary_howsmyssl_com_root_cert_pem_start");
// extern const char howsmyssl_com_root_cert_pem_end[] asm("_binary_howsmyssl_com_root_cert_pem_end");

// extern const char postman_root_cert_pem_start[] asm("_binary_postman_root_cert_pem_start");
// extern const char postman_root_cert_pem_end[] asm("_binary_postman_root_cert_pem_end");

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static char *output_buffer; // Buffer to store response of http request from event handler
    static int output_len;      // Stores number of bytes read
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
        if (!esp_http_client_is_chunked_response(evt->client))
        {
            // If user_data buffer is configured, copy the response into the buffer
            if (evt->user_data)
            {
                memcpy(evt->user_data + output_len, evt->data, evt->data_len);
            }
            else
            {
                if (output_buffer == NULL)
                {
                    output_buffer = (char *)malloc(esp_http_client_get_content_length(evt->client));
                    output_len = 0;
                    if (output_buffer == NULL)
                    {
                        ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                        return ESP_FAIL;
                    }
                }
                memcpy(output_buffer + output_len, evt->data, evt->data_len);
            }
            output_len += evt->data_len;
        }

        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        if (output_buffer != NULL)
        {
            // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
            // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
            free(output_buffer);
            output_buffer = NULL;
        }
        output_len = 0;
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        int mbedtls_err = 0;
        esp_err_t err = esp_tls_get_and_clear_last_error(evt->data, &mbedtls_err, NULL);
        if (err != 0)
        {
            if (output_buffer != NULL)
            {
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
            ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
        }
        break;
    }
    return ESP_OK;
}

typedef enum
{
    HEARTRATE,
    ACCEL,
    GYRO,
    TEMPERATURE
} API_NODES_t;

static char *createPostUrl(API_NODES_t, int, ...);

void sample_api_req_hardcoded(int deviceID, int bpm)
{
    char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};    
    API_NODES_t currentNode = TEMPERATURE;
    char *result_url = createPostUrl(currentNode, 2, 50, 500); // others

    esp_http_client_config_t config = {
        .url = result_url,
        .method = HTTP_METHOD_POST,
        .event_handler = _http_event_handler,
        .user_data = local_response_buffer // Pass address of local buffer to get response        
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
        ESP_LOGI(TAG, "It works");
    }
    else
    {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }
    ESP_LOG_BUFFER_HEX(TAG, local_response_buffer, strlen(local_response_buffer));
    free(result_url);
}

/**
 * @brief Create a Url object
 * 
 * @param api_node 
 * @param params Total Param Count. At least 2 parameter.
 * @param ...  
 * |------------------|------------|-------------|----------------|
 * | API NODE         | Param 1    | Param 2     | Param 3        |
 * |------------------|------------|-------------|----------------|
 * | HEARTRATE        | deviceID   | bpm         | null           |
 * | ACCEL            | deviceID   | acceleration| null           |
 * | GYRO             | deviceID   | angle       | angularVelocity|
 * | TEMPERATURE      | deviceID   | temperature | null           |
 * @return char[] 
 */
static char *createPostUrl(API_NODES_t api_node, int params, ...)
{    
    char *url_pointer;
    url_pointer = (char *) malloc(sizeof(char) * 100);
    strcpy(url_pointer, "http://ee5-huzza.herokuapp.com/");    
    if (params >= 0)
    {
        va_list arguments;
        va_start(arguments, params);
        int params_arr[params];

        for (int i = 0; i < params; i++)
        {
            params_arr[i] = va_arg(arguments, int);
        }

        char buffer[20];
        switch (api_node)
        {
        case HEARTRATE:
            strcat(url_pointer, "heartRate");

            if (((int)(sizeof(params_arr) / sizeof(int))) > 1)
            {
                strcat(url_pointer, "?deviceId=");
                sprintf(buffer, "%d", params_arr[0]);
                strcat(url_pointer, buffer);

                strcat(url_pointer, "&bpm=");
                sprintf(buffer, "%d", params_arr[1]);
                strcat(url_pointer, buffer);
            }

            break;
        case ACCEL:
            strcat(url_pointer, "accel");

            if (((int)(sizeof(params_arr) / sizeof(int))) > 1)
            {
                strcat(url_pointer, "?deviceId=");
                sprintf(buffer, "%d", params_arr[0]);
                strcat(url_pointer, buffer);

                strcat(url_pointer, "&acceleration=");
                sprintf(buffer, "%d", params_arr[1]);
                strcat(url_pointer, buffer);
            }

            break;
        case GYRO:
            strcat(url_pointer, "gyro");

            if (((int)(sizeof(params_arr) / sizeof(int))) > 1)
            {
                strcat(url_pointer, "?deviceId=");
                sprintf(buffer, "%d", params_arr[0]);
                strcat(url_pointer, buffer);

                strcat(url_pointer, "&angle=");
                sprintf(buffer, "%d", params_arr[1]);
                strcat(url_pointer, buffer);

                strcat(url_pointer, "&angularVelocity=");
                sprintf(buffer, "%d", params_arr[2]);
                strcat(url_pointer, buffer);
            }
            break;
        case TEMPERATURE:
            strcat(url_pointer, "temperature");

            if (((int)(sizeof(params_arr) / sizeof(int))) > 1)
            {
                strcat(url_pointer, "?deviceId=");
                sprintf(buffer, "%d", params_arr[0]);
                strcat(url_pointer, buffer);

                strcat(url_pointer, "&temperature=");
                sprintf(buffer, "%d", params_arr[1]);
                strcat(url_pointer, buffer);
            }
            break;
        default:
            // create a route for errors and send it here that
            break;
        }
    }
    
    return url_pointer;
}

static void http_test_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sending data to Temperature API");
    ESP_LOGI(TAG, "MY EXAMPLE STARTS HERE \n-------------------------------------------------------------");
    sample_api_req_hardcoded(50, 12345);
    ESP_LOGI(TAG, "MY EXAMPLE ENDS HERE \n-------------------------------------------------------------");

    //ESP_LOGI(TAG, "Finish http example");
    vTaskDelete(NULL);
}

void run(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    //ESP_ERROR_CHECK(example_connect());
    ESP_LOGI(TAG, "Connected to AP, begin http example");

    xTaskCreate(&http_test_task, "http_test_task", 8192, NULL, 5, NULL);
}

// static void sample_api_request_dynamic(void)
// {
//     char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};

//     esp_http_client_config_t config = {
//         .url = "http://ee5-huzza.herokuapp.com/heartRate?",
//         .event_handler = _http_event_handler,
//         .user_data = local_response_buffer
//     };
//     ESP_LOGI(TAG, "%s", config.url);
//     esp_http_client_handle_t client = esp_http_client_init(&config);

//     // POST
//     // const char *post_data = "{bpm=\"100\"&\"deviceId\"=50}";
//     const char *post_data = "bpm=100&deviceId=50";
//     esp_http_client_set_url(client, "http://ee5-huzza.herokuapp.com/heartRate?");
//     esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
//     esp_http_client_set_method(client, HTTP_METHOD_POST);
//     esp_http_client_set_post_field(client, post_data, strlen(post_data));

//     esp_err_t err = esp_http_client_perform(client);
//     if (err == ESP_OK)
//     {
//         ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
//                  esp_http_client_get_status_code(client),
//                  esp_http_client_get_content_length(client));
//     }
//     else
//     {
//         ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
//     }

//     esp_http_client_cleanup(client);
// }w