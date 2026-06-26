#include "global.h"

static const char *TAG_LORA = "LoRa";

static void IRAM_ATTR handle_interrupt_fromisr(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xLoraAuxSem, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR();
}

static bool lora_wait_aux_high(TickType_t timeout_ticks) {
    if (gpio_get_level(LORA_AUX))
        return true;

    if (xSemaphoreTake(xLoraAuxSem, timeout_ticks) == pdTRUE)
        return true;

    ESP_LOGW(TAG_LORA, "AUX timeout");
    return false;
}

static bool lora_send_packet(const send_t *pkt) {
    if (pkt == NULL)
        return false;

    const uint8_t *buf   = (const uint8_t *)pkt;
    const size_t   total = sizeof(send_t);

    if (!lora_wait_aux_high(pdMS_TO_TICKS(CONFIG_E220_AUX_TIMEOUT_MS))) // Waits for LoRa to be ready
    {
        ESP_LOGW(TAG_LORA, "LoRa busy before TX");
        return false;
    }

    int written = uart_write_bytes(LORA_UART_NUM, (const char *)buf, total); // Writes packet to UART at once

    if (written != total) {
        ESP_LOGE(TAG_LORA, "UART write failed (%d/%d)", written, total);
        return false;
    }

    if (!lora_wait_aux_high(pdMS_TO_TICKS(CONFIG_E220_AUX_TIMEOUT_MS))) // Waits for LoRa to finish transmission
    {
        ESP_LOGW(TAG_LORA, "TX not confirmed by AUX");
        return false;
    }

    return true;
}

static void e220_set_config(void) // E220-900T22D
{
    const uint8_t config_cmd[] = {
        0xC2, // temporary register
        0x00, // starting address
        0x08, // length
        0xFF, // ADDH
        0xFF, // ADDL, no address filtering
        0xE0, // REG0 (0b11100000: 115200 baud, 8N1, 2.4k ADR)
        0xC0, // REG1 (0b11000000: 32 bytes sub-packet, disable RSSI Ambient noise, 22dBm)
        0x41, // REG2 (850.125 + CH*1M = 915.125Mhz)
        0x00, // REG3 (0b00000000: disable RSSI byte, transparent transmission mode, disable LBT, WOR cycle not
              // applicable)
        0x00, // CRYPT_H (encryption key MSB)
        0x00, // CRYPT_L (encryption key LSB)
    };

    uart_flush(LORA_UART_NUM); // Flush UART to clear any residual data
    xSemaphoreTake(xLoraAuxSem, pdMS_TO_TICKS(200));
    uart_write_bytes(LORA_UART_NUM, (const char *)config_cmd, sizeof(config_cmd));
    xSemaphoreTake(xLoraAuxSem, pdMS_TO_TICKS(200));

    uint8_t response[sizeof(config_cmd)];
    uart_read_bytes(LORA_UART_NUM, response, sizeof(config_cmd), pdMS_TO_TICKS(100));
    if (response[0] != 0xC1)
        ESP_LOGE(TAG_LORA, "Failed to set LoRa configuration, response 0x%02X", response[0]);

    ESP_ERROR_CHECK(uart_set_baudrate(LORA_UART_NUM, LORA_BAUDRATE)); // Update baudrate after configuration
    // 8N1 is already set in uart_config
}

static void lora_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,                // E220 default baudrate is 9600
        .data_bits = UART_DATA_8_BITS,    // 8
        .parity    = UART_PARITY_DISABLE, // N
        .stop_bits = UART_STOP_BITS_1,    // 1
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(LORA_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LORA_UART_NUM, LORA_TX, LORA_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(LORA_UART_NUM, 2048, 2048, 0, NULL, 0));

    xLoraAuxSem = xSemaphoreCreateBinary();

    gpio_set_direction(LORA_AUX, GPIO_MODE_INPUT);
    gpio_pullup_en(LORA_AUX); // Enable pull-up on AUX pin
    gpio_set_intr_type(LORA_AUX, GPIO_INTR_POSEDGE);

    gpio_isr_handler_add(LORA_AUX, handle_interrupt_fromisr, NULL);

    // Set M0 and M1 to 1 for configuration mode
    gpio_set_direction(LORA_M0, GPIO_MODE_OUTPUT);
    gpio_set_direction(LORA_M1, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_M0, 1);
    gpio_set_level(LORA_M1, 1);
    vTaskDelay(pdMS_TO_TICKS(50)); // Short delay

    e220_set_config();

    // Set M0 and M1 to 0 for normal mode
    gpio_set_level(LORA_M0, 0);
    gpio_set_level(LORA_M1, 0);
    vTaskDelay(pdMS_TO_TICKS(50)); // Short delay

    ESP_LOGI(TAG_LORA, "LoRa UART initialized (baud %d)", LORA_BAUDRATE);
}

void task_lora(void *pvParameters) {
    lora_init();

    send_t     send_data;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        xQueuePeek(xLoraQueue, &send_data, 0); // Non-blocking peek, will use last data if queue hasn't been updated yet

        if (!lora_send_packet(&send_data))
            ESP_LOGW(TAG_LORA, "Failed to send LoRa packet — discarded");

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LORA_RATE_MS));
    }
}