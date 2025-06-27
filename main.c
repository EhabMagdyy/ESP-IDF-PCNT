/*************************************************************************************************************
***************************    Author  : Ehab Magdy Abdullah                      ****************************
***************************    Linkedin: https://www.linkedin.com/in/ehabmagdyy/  ****************************
***************************    Youtube : https://www.youtube.com/@EhabMagdyy      ****************************
**************************************************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Function declarations
void PulseCounter_Task(void *pvParameter);
static bool IRAM_ATTR pcnt_event_callback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);

#define TAG                 "PCNT_EXAMPLE"

#define PCNT_GPIO           GPIO_NUM_19
#define LED_PIN             GPIO_NUM_2

#define EVENT_QUEUE_LEN     10      // Queue Length
#define PCNT_HIGH_LIMIT     21
#define PCNT_LOW_LIMIT      -20

#define PCNT_WATCHPOINT_1   10      // Watchpoint 1 at 10 counts
#define PCNT_WATCHPOINT_2   20      // Watchpoint 2 at 20 counts

static pcnt_unit_handle_t pcnt_unit = NULL;
static QueueHandle_t pcnt_event_queue = NULL;

// Event structure to pass data from ISR to task
typedef struct {
    int watch_point_value;
    pcnt_unit_zero_cross_mode_t zero_cross_mode;
} pcnt_event_t;

void app_main(void)
{
    xTaskCreate(PulseCounter_Task, "PulseCounter_Task", 4096, NULL, 5, NULL);
}

void PulseCounter_Task(void *pvParameter)
{
    ESP_LOGI(TAG, "Pulse Counter Task Started!");

    // 1. Configure PCNT unit
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    // 2. Configure PCNT channel
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = PCNT_GPIO,
        .level_gpio_num = -1,  // Not using level control
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    // 3. Configure edge actions
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        pcnt_chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,  // Count rising edge
        PCNT_CHANNEL_EDGE_ACTION_HOLD       // No action on falling edge
    ));

    // 4. Register callbacks
    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_event_callback,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, NULL));

    // 5. Add watch points
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, PCNT_WATCHPOINT_1));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, PCNT_WATCHPOINT_2));

    // 6. Configure glitch filter
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 10000,    // 10us
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    // 7. Enable and start Counter
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    // Create event queue to receive data from ISR
    pcnt_event_queue = xQueueCreate(EVENT_QUEUE_LEN, sizeof(pcnt_event_t));

    pcnt_event_t event;
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    for(;;)
    {
        // Process events from queue
        if (xQueueReceive(pcnt_event_queue, &event, pdMS_TO_TICKS(1))) {
            ESP_LOGI(TAG, "--->> Event: Watch Point = %d", event.watch_point_value);
            if(event.watch_point_value == PCNT_WATCHPOINT_1)
                gpio_set_level(LED_PIN, 1);
            else if (event.watch_point_value == PCNT_WATCHPOINT_2)
                gpio_set_level(LED_PIN, 0);
        }

        // Periodic count read
        int pulse_count = 0;
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
        ESP_LOGI(TAG, "Current pulse count: %d", pulse_count);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ISR callback function
static bool IRAM_ATTR pcnt_event_callback(pcnt_unit_handle_t unit,
    const pcnt_watch_event_data_t *edata,
    void *user_ctx)
{
    BaseType_t high_task_wakeup = pdFALSE;

    // Create event data
    pcnt_event_t event = {
        .watch_point_value = edata->watch_point_value,
        .zero_cross_mode = edata->zero_cross_mode
    };

    // Send event data to queue (ISR safe)
    xQueueSendFromISR(pcnt_event_queue, &event, &high_task_wakeup);

    return high_task_wakeup == pdTRUE;
}