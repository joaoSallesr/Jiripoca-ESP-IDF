#include "acquire.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define SEA_LEVEL_PRESSURE_HPA 1013.25f
#define ISA_ALTITUDE_FACTOR 44330.0f
#define ISA_PRESSURE_EXPONENT 0.00019029496f // 1.0f / 5.255f

static const char *TAG_ACQ = "Acquire";
static const char *TAG_ICM = "ICM20948";
static const char *TAG_BMP = "BMP390";

i2c_master_bus_handle_t bus_handle = NULL;
static bmp390_handle_t bmp_dev_hdl;

// standard i2c bus config
i2c_master_bus_config_t bus_config = {
    .i2c_port = I2C_MASTER_NUM,
    .sda_io_num = I2C_SDA,
    .scl_io_num = I2C_SCL,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

void adc_init(adc_oneshot_unit_handle_t *adc_unit_handle, adc_cali_handle_t *adc_cali_handle) {
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    adc_oneshot_new_unit(&unit_config, adc_unit_handle);

    // Configure the ADC channel
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,             
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc_unit_handle, ADC_CHANNEL_4, &channel_config));

    // Configure calibration (raw ADC value in mV)
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_4,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, adc_cali_handle));
}

float read_battery_voltage(adc_oneshot_unit_handle_t adc_unit_handle, adc_cali_handle_t adc_cali_handle) {
    int raw;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_unit_handle, ADC_CHANNEL_4, &raw));

    int voltage_mv;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv));

    // Calculate actual battery voltage based on voltage divider ratio
    float battery_voltage = (voltage_mv / 1000.0f) * ((R1 + R2) / R2); // Convert mV to V and apply divider ratio
    // (R1 + R2) / R2 = 1.5 for R1=10k and R2=20k
    return battery_voltage;
}

static float lat_lon_conversion(float ddmm) {
    int deg = (int)(ddmm / 100.0f);
    float min = ddmm - (deg * 100.0f);
    return deg + (min / 60.0f);
}
// since the GPGGA sentence gives us latitude/longitude in ddmm.mmmm values,
// it was necessary to create a function that would transform this into decimal values

static bool parse_gpgga_line(char *line, data_t *data) {
    if (strncmp(line, "$GPGGA", 6) != 0){
        return false;
    }
    // this will check if the sentence received by the gps is $GPGGA

    char *fields[15] = {0}; 
    size_t field_count = 0; // auxiliary variable used by strtok_r.
    // necessary because strtok_r needs context to remember where it left off.
    char *context = NULL;    

    for (char *field = strtok_r(line, ",", &context);
         field && field_count < 15;
         field = strtok_r(NULL, ",", &context)) 
    {
        fields[field_count++] = field;
    }
    // strtok_r is from the string library, it will break the string into diferent tokens/"fields"
    // each field is a GPS information

    // latitude (field 2 + field 3)
    if (fields[2] && *fields[2] && fields[3]) {
        float raw_lat = atof(fields[2]);
        data->latitude = lat_lon_conversion(raw_lat);
        if (fields[3][0] == 'S') data->latitude = -data->latitude;
        // the coordinates are negative at south and west
    }
    // longitude (field 4 + field 5)
    if (fields[4] && *fields[4] && fields[5]) {
        float raw_lon = atof(fields[4]);
        data->longitude = lat_lon_conversion(raw_lon);
        if (fields[5][0] == 'W') data->longitude = -data->longitude;
    }
    // altitude (field 9)
    if (fields[9] && *fields[9]) {
        data->gps_altitude = atof(fields[9]);
    }

    return true;
}

void task_gps(void *pvParameters){

    data_t *gps_data = (data_t *)pvParameters;

    uart_port_t uart_num = UART_NUM_1;
    const int uart_buffer_size = 2048;
    QueueHandle_t uart_queue;
    uart_config_t gps_uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(uart_num, &gps_uart_config);

    uart_set_pin(uart_num, GPS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 20, &uart_queue, 0);

    char line[128];
    size_t line_len = 0;
    uint8_t uart_rx[64];

    while (true) {
        // it starts readding the data from the gps
        // until it finds a newline character, which indicates the end of a sentence
        int len = uart_read_bytes(uart_num, uart_rx, sizeof(uart_rx), pdMS_TO_TICKS(1000));
        if (len > 0) {
            line_len += len;
            line[line_len] = '\0'; // null terminate the string

            // look for newline characters to identify complete lines
            char *start = line;
            char *newline;
            while ((newline = strchr(start, '\n')) != NULL) {
                *newline = '\0'; // replace newline with null terminator
                if (parse_gpgga_line(start, gps_data)){
                    // if the line is a valid GPGGA sentence, update gps_data
                    // ESP_LOGI(TAG_ACQ, "GPS Data - Lat: %.5f, Lon: %.5f, Alt: %.2f", gps_data->latitude, gps_data->longitude, gps_data->gps_altitude);
                    // successfully parsed a GPGGA line
                }
                start = newline + 1; // move to the start of the next line
            }
        size_t remaining = line_len - (start - line);
        // calculate how many bytes remain after the last processed '\n'
        memmove(line, start, remaining);
        // shift the remaining bytes to the beginning of the buffer
        // this ensures that incomplete data is kept for the next UART read
        line_len = remaining;
        // update the line length to match the number of leftover bytes
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    free(line);
    uart_driver_delete(uart_num);
}

void init_icm20948(icm20948_device_t *icm)
{
    i2c_master_dev_handle_t icm_handle;
    
    i2c_device_config_t icm_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM_20948_I2C_ADDR_AD0,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &icm_dev_cfg, &icm_handle));

    // standard ICM20948 config
    icm0948_config_i2c_t icm_config = {
        .i2c_dev = icm_handle,
        .i2c_addr = ICM_20948_I2C_ADDR_AD0
    };


    // ICM20948 scale range config 
    icm20948_fss_t myfss = {.a = GPM_16,
                            .g = DPS_500, 

    };

    // setup ICM20948
    icm20948_init_i2c(icm, &icm_config);
    
    // reset device state
    icm20948_sw_reset(icm);
	vTaskDelay(pdMS_TO_TICKS(250));

    icm20948_internal_sensor_id_bm sensors = (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);
    icm20948_set_sample_mode(icm, sensors, SAMPLE_MODE_CONTINUOUS); 

    // wake up sensor with scale applied
    icm20948_sleep(icm, false);
	icm20948_low_power(icm, false);
    icm20948_set_full_scale(icm, sensors, myfss);
    vTaskDelay(pdMS_TO_TICKS(250));
}

void acquire_icm20948(data_t *data, icm20948_device_t *icm, icm20948_agmt_t *agmt)
{
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    if (icm20948_get_agmt(icm, agmt) != ICM_20948_STAT_OK) 
        ESP_LOGE(TAG_ICM, "Failed to read ICM20948");
    xSemaphoreGive(xI2CMutex);

    // acquire data and apply scale factor
    data->accel_x = agmt->acc.axes.x/ICM_SCALE_16G;
    data->accel_y = agmt->acc.axes.y/ICM_SCALE_16G;
    data->accel_z = agmt->acc.axes.z/ICM_SCALE_16G;
    data->rotation_x = agmt->gyr.axes.x/ICM_SCALE_500DPS;
    data->rotation_y = agmt->gyr.axes.y/ICM_SCALE_500DPS;
    data->rotation_z = agmt->gyr.axes.z/ICM_SCALE_500DPS;
    data->magnet_x = agmt->mag.axes.x/ICM_SCALE_4900μT;
    data->magnet_y = agmt->mag.axes.y/ICM_SCALE_4900μT;
    data->magnet_z = agmt->mag.axes.z/ICM_SCALE_4900μT;

    vTaskDelay(0);
}

esp_err_t bmp_init(void)
{
    i2c_master_dev_handle_t bmp_handle;

    i2c_device_config_t bmp_i2c_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP390_I2C_ADDRESS,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &bmp_i2c_cfg, &bmp_handle));

    bmp390_config_t bmp_cfg = I2C_BMP390_CONFIG_DEFAULT;
    bmp_cfg.i2c_address = BMP390_I2C_ADDRESS;

    ESP_ERROR_CHECK(bmp390_init(bus_handle, &bmp_cfg, &bmp_dev_hdl));
    ESP_ERROR_CHECK(bmp390_set_iir_filter(bmp_dev_hdl, BMP390_IIR_FILTER_3));
    return ESP_OK;
}

void acquire_bmp390(data_t* data, bmp390_handle_t dev_hdl)
{
    ESP_LOGI(TAG_BMP, "######################## BMP390 - START #########################");

    // sensor readings
    ESP_ERROR_CHECK(bmp390_get_measurements(bmp_dev_hdl, &data->temperature, &data->pressure));
    
    data->pressure = data->pressure / 100.0f;
    ESP_LOGI(TAG_BMP, "air temperature:      %.2f °C", data->temperature);
    ESP_LOGI(TAG_BMP, "barometric pressure: %.2f hPa", data->pressure);

    float temp_altitude = 0;
    // BMP390 altitude calculation (barometric formula)
    temp_altitude = ISA_ALTITUDE_FACTOR * (1.0f - powf(data->pressure / SEA_LEVEL_PRESSURE_HPA, ISA_PRESSURE_EXPONENT));

    if (temp_altitude > data->max_altitude)
        data->max_altitude = temp_altitude;
    data->bmp_altitude = temp_altitude;
    
    ESP_LOGI(TAG_BMP, "######################## BMP390 - END ###########################");
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
        xQueueSend(xLoraQueue, data, 0); // Send to LoRa queue
    }
}

void task_acquire(void *pvParameters)
{
    xI2CMutex = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    data_t data = {0};

    // ICM20948 
    icm20948_device_t icm;
    icm20948_agmt_t agmt;
    init_icm20948(&icm);

    // BMP390
    bmp_init();

    xTaskCreate(task_gps, "GPS", configMINIMAL_STACK_SIZE * 4, &data, 6, NULL);

    adc_oneshot_unit_handle_t adc_unit_handle;
    adc_cali_handle_t adc_cali_handle;
    adc_init(&adc_unit_handle, &adc_cali_handle);

    vTaskDelay(pdMS_TO_TICKS(1000));
    while (true)
    {
        // Time and status update
        data.time = (int32_t)(esp_timer_get_time() / 1000);
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        data.status = STATUS;
        xSemaphoreGive(xStatusMutex);

        // Battery voltage
        data.voltage = read_battery_voltage(adc_unit_handle, adc_cali_handle);

        // ICM20948
        acquire_icm20948(&data, &icm, &agmt);

        // BMP390
        acquire_bmp390(&data, bmp_dev_hdl);

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
    // bmp390_delete(dev_hdl);
    vTaskDelete(NULL);
}