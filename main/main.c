#include "global.h"

static const char *TAG_MAIN = "MAIN";

#define STATUS_QUEUE_SIZE   20
#define SD_QUEUE_SIZE       80
#define LITTLEFS_QUEUE_SIZE 80
#define LORA_QUEUE_SIZE     1   // Overwrite queue
#define B4LAUNCH_QUEUE_SIZE 650 // Max of 13 samples/s: 650 =~ 5s of flight data

static bool check_for_format_mode(void) {
    if (gpio_get_level(BUTTON_GPIO) == LOW) {
        int64_t time = esp_timer_get_time();
        while (gpio_get_level(BUTTON_GPIO) == LOW) {
            if (esp_timer_get_time() - time > 5000000LL) {
                ESP_LOGW("RESET", "Button pressed for 5 seconds. Formatting...");
                // Signal format mode
                for (uint32_t i = 0; i < 2; ++i) {
                    gpio_set_level(LED_GPIO, HIGH);
                    gpio_set_level(BUZZER_GPIO, HIGH);
                    vTaskDelay(pdMS_TO_TICKS(150));
                    gpio_set_level(LED_GPIO, LOW);
                    gpio_set_level(BUZZER_GPIO, LOW);
                    vTaskDelay(pdMS_TO_TICKS(150));
                }
                return true;
            }
            vTaskDelay(10);
        }
    }
    return false;
}

void app_main(void) {
    ESP_LOGI(TAG_MAIN, "Starting main application");

    /* Create Queue */
    xEventQueue    = xQueueCreate(STATUS_QUEUE_SIZE, sizeof(status_event_t));
    xSDQueue       = xQueueCreate(SD_QUEUE_SIZE, sizeof(save_t));
    xLittleFSQueue = xQueueCreate(LITTLEFS_QUEUE_SIZE, sizeof(save_t));
    xLoraQueue     = xQueueCreate(LORA_QUEUE_SIZE, sizeof(send_t));
    xB4LaunchQueue = xQueueCreate(B4LAUNCH_QUEUE_SIZE, sizeof(save_t));

    /* Create Mutex */
    xI2CSem = xSemaphoreCreateMutex();

    /* Create Event Group */
    xStatusEventGroup     = xEventGroupCreate();
    xInitEventGroup       = xEventGroupCreate();
    xNVSCounterEventGroup = xEventGroupCreate();
    xFormatEventGroup     = xEventGroupCreate();

    /* Setup Tasks */
    xTaskCreatePinnedToCore(task_setup, "Setup", configMINIMAL_STACK_SIZE * 8, NULL, 10, NULL, 1);

    const EventBits_t bits_to_wait = EVT_SD_DONE | EVT_LFS_DONE;

    do {
        bool format_mode = check_for_format_mode();
        manage_nvs_counters(format_mode);
        // If format is true, format SD and LittleFS, then restart
        if (format_mode) {
            xTaskCreate(task_sd, "SD", configMINIMAL_STACK_SIZE * 8, &file_counter_g, 5, NULL);
            xTaskCreate(task_lfs, "LittleFS", configMINIMAL_STACK_SIZE * 8, &file_counter_g, 5, NULL);
            xEventGroupWaitBits(xFormatEventGroup, bits_to_wait, pdTRUE, pdTRUE, portMAX_DELAY);
            ESP_LOGW(TAG_MAIN, "Restarting after format...");
            esp_restart();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    } while (gpio_get_level(RBF_GPIO) == LOW); // While not armed

    data_g.status |= ARMED;

    // Create tasks
    xTaskCreatePinnedToCore(task_gps, "GPS", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_bmp, "BMP", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_fusion, "ICM", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_acquire, "ACQUIRE", configMINIMAL_STACK_SIZE * 4, NULL, 4, &xTaskAcquire, 0);
    xTaskCreatePinnedToCore(task_sd, "SD", configMINIMAL_STACK_SIZE * 8, &file_counter_g, 3, NULL, 0);
    xTaskCreatePinnedToCore(task_lfs, "LittleFS", configMINIMAL_STACK_SIZE * 8, &file_counter_g, 3, NULL, 0);
    xTaskCreatePinnedToCore(task_buzzer_led, "BUZZER LED", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(task_lora, "LORA", configMINIMAL_STACK_SIZE * 2, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(task_adc, "ADC", configMINIMAL_STACK_SIZE * 2, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_nvs, "NVS", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL, 0);
#if CONFIG_LOG_DEFAULT_LEVEL >= ESP_LOG_DEBUG
    xTaskCreatePinnedToCore(task_log, "LOG", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL, 0);
#endif
}