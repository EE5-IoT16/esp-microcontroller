#include "pulse_counter.h"
static const char *TAG = "pulse_counter";

#define PCNT_H_LIM_VAL      20
#define PCNT_L_LIM_VAL     -10
#define PCNT_THRESH1_VAL    16
#define PCNT_THRESH0_VAL   -5
#define PCNT_INPUT_SIG_IO   4  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  5  // Control GPIO HIGH=count up, LOW=count down
#define LEDC_OUTPUT_IO      18 // Output GPIO of a sample 1 Hz pulse generator

int firstCount = 0;
uint16_t heartRate  = 0;
esp_err_t err;
nvs_handle_t my_handle;

xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
int pcnt_unit;

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
    int pcnt_unit = (int)arg;
    pcnt_evt_t evt;
    evt.unit = pcnt_unit;
    /* Save the PCNT event type that caused an interrupt
       to pass it to the main program */
    pcnt_get_event_status(pcnt_unit, &evt.status);
    xQueueSendFromISR(pcnt_evt_queue, &evt, NULL);
}

/* Configure LED PWM Controller
 * to output sample pulses at 1 Hz with duty of about 10%
 */
static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num        = LEDC_TIMER_1;
    ledc_timer.duty_resolution  = LEDC_TIMER_10_BIT;
    ledc_timer.freq_hz          = 1;  // set output frequency at 1 Hz
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel    = LEDC_CHANNEL_1;
    ledc_channel.timer_sel  = LEDC_TIMER_1;
    ledc_channel.intr_type  = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num   = LEDC_OUTPUT_IO;
    ledc_channel.duty       = 500; // set duty at about 10%
    ledc_channel.hpoint     = 0;
    ledc_channel_config(&ledc_channel);
}

/* Initialize PCNT functions:
 *  - configure and initialize PCNT
 *  - set up the input filter
 *  - set up the counter events to watch
 */
static void pcnt_example_init(int unit)
{
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = unit,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(unit,50);
    pcnt_filter_enable(unit);

    /* Set threshold 0 and 1 values and enable events to watch */
    pcnt_set_event_value(unit, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
   // pcnt_event_enable(unit, PCNT_EVT_THRES_1);
    pcnt_set_event_value(unit, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    //pcnt_event_enable(unit, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    //pcnt_event_enable(unit, PCNT_EVT_ZERO);
    //pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    //pcnt_event_enable(unit, PCNT_EVT_L_LIM);
    /* Initialize PCNT's counter */
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);

    /* Install interrupt service and add isr callback handler */
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(unit, pcnt_example_intr_handler, (void *)unit);
    /* Everything is set up, now go to counting */
    pcnt_counter_resume(unit);
}

void counter_init(void)
{
    pcnt_unit = PCNT_UNIT_0;
    /* Initialize LEDC to generate sample pulse signal */
    ledc_init();
    /* Initialize PCNT event queue and PCNT functions */
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_example_init(pcnt_unit);
    
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) 
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err)); 
    }
}


HeartRateStatus startToCount(int period)
{   
    int16_t count = 0;
    HeartRateStatus status = NORMAL;
    pcnt_evt_t evt;
    portBASE_TYPE res;
    /* Wait for the event information passed from PCNT's interrupt handler.
     * Once received, decode the event type and print it on the serial monitor.
     */
    pcnt_counter_pause(pcnt_unit);
    pcnt_counter_clear(pcnt_unit);
    pcnt_counter_resume(pcnt_unit); 
    res = xQueueReceive(pcnt_evt_queue, &evt, period / portTICK_PERIOD_MS);
    pcnt_get_counter_value(pcnt_unit, &count);
    if (firstCount == 0)
    {
        firstCount =1;
        //count++;
        count = count*2;

    }
    if (res == pdTRUE) {
        ESP_LOGI(TAG, "Event PCNT unit[%d]; cnt: %d", evt.unit, count);
        if (evt.status & PCNT_EVT_THRES_1) {
            ESP_LOGI(TAG, "THRES1 EVT");
        }
        if (evt.status & PCNT_EVT_THRES_0) {
            ESP_LOGI(TAG, "THRES0 EVT");
        }
        if (evt.status & PCNT_EVT_L_LIM) {
            ESP_LOGI(TAG, "L_LIM EVT");   
            }
        if (evt.status & PCNT_EVT_H_LIM) {
            ESP_LOGI(TAG, "H_LIM EVT");
        }
        if (evt.status & PCNT_EVT_ZERO) {
            ESP_LOGI(TAG, "ZERO EVT");
        }
        } 
        else { 
            heartRate += (uint16_t)count*12;
            heartRate /=2;
            ESP_LOGI(TAG, "Current counter value :%d, heart rate :%d bps", count,heartRate);
            status = NORMAL;
            }

        err = nvs_set_u16(my_handle, "bpm", heartRate);
        err = nvs_commit(my_handle);
        return status;
}

void unitializePulseCounter(void)
{
    nvs_close(my_handle);
}
