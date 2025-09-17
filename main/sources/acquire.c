#include "acquire.h"

static const char *TAG_ACQ = "Acquire";
static const float SEA_LEVEL_PRESSURE_HPA = 1013.25f;

void init_bmp390(bmp390_config_t* dev_cfg, bmp390_handle_t* dev_hdl)
{
    // init device
    bmp390_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "bmp390 handle init failed");
        assert(dev_hdl);
    }

    /* configuration registers */
    bmp390_power_control_register_t     power_ctrl_reg;
    bmp390_configuration_register_t     config_reg;
    bmp390_oversampling_register_t      oversampling_reg;
    bmp390_output_data_rate_register_t  output_data_rate_reg;
    bmp390_interrupt_control_register_t interrupt_ctrl_reg;

    /* attempt to read configuration register */
    bmp390_get_configuration_register(dev_hdl, &config_reg);

    /* attempt to read oversampling register */
    bmp390_get_oversampling_register(dev_hdl, &oversampling_reg);

    /* attempt to read to power control register */
    bmp390_get_power_control_register(dev_hdl, &power_ctrl_reg);

    /* attempt to read to output data rate register */
    bmp390_get_output_data_rate_register(dev_hdl, &output_data_rate_reg);

    /* attempt to read to interrupt control register */
    bmp390_get_interrupt_control_register(dev_hdl, &interrupt_ctrl_reg);


    ESP_LOGI(APP_TAG, "Configuration (0x%02x): %s", config_reg.reg,           uint8_to_binary(config_reg.reg));
    ESP_LOGI(APP_TAG, "Oversampling  (0x%02x): %s", oversampling_reg.reg,     uint8_to_binary(oversampling_reg.reg));
    ESP_LOGI(APP_TAG, "Data Rate     (0x%02x): %s", output_data_rate_reg.reg, uint8_to_binary(output_data_rate_reg.reg));
    ESP_LOGI(APP_TAG, "Power Control (0x%02x): %s", power_ctrl_reg.reg,       uint8_to_binary(power_ctrl_reg.reg));
    ESP_LOGI(APP_TAG, "Int Control   (0x%02x): %s", interrupt_ctrl_reg.reg,   uint8_to_binary(interrupt_ctrl_reg.reg));

    if(interrupt_ctrl_reg.bits.irq_data_ready_enabled)
        ESP_LOGE(APP_TAG, "bmp390 irq data ready is enabled");
}

void acquire_bmp390(data_t *data, bmp390_handle_t* dev_hdl)
{
    ESP_LOGI(APP_TAG, "######################## BMP390 - START #########################");

    // sensor readings
    esp_err_t result = bmp390_get_measurements(dev_hdl, &data->temperature, &data->pressure);
    
    if (result != ESP_OK)
    {
        ESP_LOGE(APP_TAG, "bmp390 device read failed (%s)", esp_err_to_name(result));
    }
    else 
    {
        data->pressure = data->pressure / 100.0f;
        ESP_LOGI(APP_TAG, "air temperature:      %.2f °C", data->temperature);
        ESP_LOGI(APP_TAG, "barometric pressure: %.2f hPa", data->pressure);
    }

    float temp_altitude = 0;
    // BMP390 altitude calculation (barometric formula)
    temp_altitude = 44330.0f * (1.0f - powf(data->pressure / SEA_LEVEL_PRESSURE_HPA, 1.0f / 5.255f));

    if (temp_altitude > data->max_altitude)
        data->max_altitude = temp_altitude;
    data->bmp_altitude = temp_altitude;
    
    ESP_LOGI(APP_TAG, "######################## BMP390 - END ###########################");
}

// status_checks checks if the rocket is flying, motor is cutoff, or landed
void status_checks(data_t *data)
{
    // Check if accel is higher than FLYING_THRESHOLD
    if (!(data->status & FLYING))
    {
        if (fabs(data->accel_x) > FLYING_THRESHOLD || fabs(data->accel_y) > FLYING_THRESHOLD || fabs(data->accel_z) > FLYING_THRESHOLD)
        {
            xSemaphoreTake(xStatusMutex, portMAX_DELAY);
            STATUS |= FLYING;
            xSemaphoreGive(xStatusMutex);
        }
    }

    // Check if accel is lower than CUTOFF_THRESHOLD
    if ((data->status & FLYING) && !(data->status & CUTOFF))
    {
        if (fabs(data->accel_x) < CUTOFF_THRESHOLD && fabs(data->accel_y) < CUTOFF_THRESHOLD && fabs(data->accel_z) < CUTOFF_THRESHOLD)
        {
            xSemaphoreTake(xStatusMutex, portMAX_DELAY);
            STATUS |= CUTOFF;
            xSemaphoreGive(xStatusMutex);
        }
    }

    // Check if landed by comparing altitude to altitude 5 seconds ago
    if ((data->status & FLYING) && !(data->status & LANDED))
    {
        static float aux_altitude = 0;
        static int64_t aux_time = 0;
        if (esp_timer_get_time() - aux_time > 5000000) // If 5 seconds have passed since last check
        {
            if (aux_time == 0) // If first time checking, set aux_time and aux_altitude
            {
                aux_time = esp_timer_get_time();
                aux_altitude = data->bmp_altitude;
            }
            else if (fabs(data->bmp_altitude - aux_altitude) < LANDED_THRESHOLD) // If altitude has not changed more than LANDED_THRESHOLD, consider landed
            {
                xSemaphoreTake(xStatusMutex, portMAX_DELAY);
                STATUS |= LANDED;
                xSemaphoreGive(xStatusMutex);
            }
            else // If altitude has changed more than LANDED_THRESHOLD, update aux_time and aux_altitude
            {
                aux_time = esp_timer_get_time();
                aux_altitude = data->bmp_altitude;
            }
        }
    }
}
// send_queues sends data to queues
void send_queues(data_t *data)
{
    if (!(data->status & LANDED)) // If not landed, send to queues
    {
        if ((data->status & ARMED)) // If armed, send to task_deploy
            xQueueSend(xAltQueue, &data->bmp_altitude, 0);
        xQueueSend(xSDQueue, data, 0);  // Send to SD card queue
        if (!(data->status & LFS_FULL)) // If LittleFS is not full, send to LittleFS queue
            xQueueSend(xLittleFSQueue, data, 0);
    }

    // ESP_LOGI("Acquire", "Data sent to queues");
}

void task_acquire(void *pvParameters)
{
    xI2CMutex = xSemaphoreCreateMutex();

    data_t data = {0};

    // init bmp390
    bmp390_config_t dev_cfg = I2C_BMP390_CONFIG_DEFAULT;
    bmp390_handle_t dev_hdl;
    init_bmp390(&dev_cfg, &dev_hdl);


    vTaskDelay(pdMS_TO_TICKS(1000));
    while (true)
    {
        // Time and status update
        data.time = (int32_t)(esp_timer_get_time() / 1000);
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        data.status = STATUS;
        xSemaphoreGive(xStatusMutex);

        acquire_bmp390(&data, &dev_bmp);

        status_checks(&data);

        // Print data
        ESP_LOGI(TAG_ACQ, "\tTime: %ld, Status: %ld V: %.2f\r\n"
                          "\tBMP\t\tP: %.2f, T: %.2f, A: %.2f\r\n"
                          "\tAccel\t\tX: %.2f, Y: %.2f, Z: %.2f\r\n"
                          "\tGyro\t\tH: %.2f, P: %.2f, Y: %.2f\r\n"
                          "\tGPS\t\tLat: %.5f, Lon: %.5f, A-GPS: %.2f",
                 data.time, data.status, data.voltage,
                 data.pressure, data.temperature, data.bmp_altitude,
                 data.accel_x, data.accel_y, data.accel_z,
                 data.rotation_x, data.rotation_y, data.rotation_z,
                 data.latitude, data.longitude, data.gps_altitude);

        send_queues(&data);

        // REDUCE AFTER OPTIMIZING CODE
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    bmp390_delete(dev_hdl);
    vTaskDelete(NULL);
}