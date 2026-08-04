#include "global.h"

static const char *TAG_LORA = "LoRa";

/* LORA CONFIG */
#define LORA_FREQUENCY        915000000 // Hz
#define LORA_SPREADING_FACTOR 5
#define LORA_BANDWIDTH        SX126X_LORA_BW_500_0
#define LORA_CODING_RATE      SX126X_LORA_CR_4_5
#define LORA_DIO1_TIMEOUT_MS  1000
#define LORA_RATE_MS          200 // 5Hz

static SemaphoreHandle_t xLoraDio1Sem;

static void IRAM_ATTR lora_dio1_isr(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xLoraDio1Sem, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR();
}

static bool lora_wait_dio1(sx126x_handle_t handle) {
    if (xSemaphoreTake(xLoraDio1Sem, pdMS_TO_TICKS(LORA_DIO1_TIMEOUT_MS)) == pdTRUE) {
        return true;
    }

    ESP_LOGE(TAG_LORA, "DIO1 semaphore timeout");
    handle->txActive = false;
    ClearIrqStatus(handle, SX126X_IRQ_ALL);
    return false;
}

static bool lora_send_packet(sx126x_handle_t handle, const send_t *pkt) {
    if (pkt == NULL)
        return false;

    if (!LoRaSendAsync(handle, (uint8_t *)pkt, sizeof(send_t))) {
        ESP_LOGW(TAG_LORA, "TX busy — skipping packet");
        return false;
    }

    if (!lora_wait_dio1(handle))
        return false;

    uint16_t irq;
    return LoRaTxWaitDone(handle, &irq);
}

static esp_err_t lora_init(sx126x_handle_t *lora_handle) {
    esp_err_t err = ESP_OK;

    /* SX1262 LoRa struct setup */
    sx126x_config_t lora_cfg = {
        .spi_host          = SPI_HOST,
        .ss                = LORA_CS,
        .reset             = LORA_RESET,
        .busy              = LORA_BUSY,
        .txen              = -1,
        .rxen              = -1,
        .frequency         = LORA_FREQUENCY,
        .tx_power          = 22,
        .tcxo_voltage      = 0.0f,
        .use_regulator_ldo = false,
        .spreading_factor  = LORA_SPREADING_FACTOR,
        .bandwidth         = LORA_BANDWIDTH,
        .coding_rate       = LORA_CODING_RATE,
        .preamble_length   = 12,    // 8 -> SF7-8 || 12 -> SF5-6
        .payload_len       = 0,     // Variable length packet
        .crc_on            = true,  // true -> drop garbage
        .invert_iq         = false, // false -> normal communication
    };

    /* SX1262 LoRa initialization */
    err = LoRaInit(&lora_cfg, lora_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_LORA, "LoRa init failed: %s", esp_err_to_name(err));
        return err;
    }

    LoRaDebugPrint(*lora_handle, false);
    int16_t lora_ret = LoRaBegin(*lora_handle);
    if (lora_ret != ERR_NONE) {
        ESP_LOGE(TAG_LORA, "LoRa begin failed: %d", lora_ret);
        return ESP_FAIL;
    }

    LoRaConfig(*lora_handle);

    /* SX1262 Interrupt Gpio */
    xLoraDio1Sem = xSemaphoreCreateBinary();
    gpio_set_direction(LORA_DIO1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LORA_DIO1, GPIO_FLOATING);
    gpio_set_intr_type(LORA_DIO1, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(LORA_DIO1, lora_dio1_isr, NULL);

    /* IRQ parameters */
    uint16_t irqMask  = SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT;
    uint16_t dio1Mask = SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT;
    SetDioIrqParams(*lora_handle, irqMask, dio1Mask, 0, 0);

    ESP_LOGI(TAG_LORA, "LoRa initialized");
    return ESP_OK;
}

void task_lora(void *pvParameters) {
    esp_err_t       err;
    sx126x_handle_t lora_handle;

    send_t send_data = {0};

    TickType_t xLastWakeTime = xTaskGetTickCount();

    err = lora_init(&lora_handle);
    if (err != ESP_OK) {
        goto setup_error;
    }

    while (true) {
        xQueuePeek(xLoraQueue, &send_data, 0); // Non-blocking peek, will use last data if queue hasn't been updated yet

        if (!lora_send_packet(lora_handle, &send_data))
            ESP_LOGW(TAG_LORA, "Failed to send LoRa packet — discarded");

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LORA_RATE_MS));
    }

cleanup:
    gpio_intr_disable(LORA_DIO1);
    gpio_isr_handler_remove(LORA_DIO1);

    vTaskDelete(NULL);

setup_error: {
    ESP_LOGE(TAG_LORA, "LoRa init failed: %s", esp_err_to_name(err));

    status_event_t evt = EVT_SETUP_FAILED;
    xQueueSend(xEventQueue, &evt, portMAX_DELAY);
}

    goto cleanup;
}