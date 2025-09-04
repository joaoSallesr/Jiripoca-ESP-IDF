#include "acquire.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

static const char *TAG_ACQ = "Acquire";
static const char *TAG_ICM = "ICM20948";

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

void init_icm20948(icm20948_device_t *icm)
{

    // standard ic2 bus config from component example
    i2c_config_t bus_config = { .mode = I2C_MODE_MASTER,
	                            .sda_io_num = (gpio_num_t) I2C_SDA,
	                            .sda_pullup_en = GPIO_PULLUP_ENABLE,
	                            .scl_io_num = (gpio_num_t) I2C_SCL,
	                            .scl_pullup_en = GPIO_PULLUP_ENABLE,
	                            .master.clk_speed = I2C_MASTER_FREQ_HZ,
	                            .clk_flags = 0 };

    // standard ICM20948 config from component example
    icm0948_config_i2c_t icm_config = { .i2c_port = I2C_MASTER_NUM,
	                                    .i2c_addr = ICM_20948_I2C_ADDR_AD1 };

    // setup i2c
	ESP_ERROR_CHECK(i2c_param_config(icm_config.i2c_port, &bus_config));
	ESP_ERROR_CHECK(i2c_driver_install(icm_config.i2c_port, bus_config.mode, 0, 0, 0));

    // setup ICM20948
    icm20948_init_i2c(icm, &icm_config);

    // reset device state -> test later to see if it's really necessary
    icm20948_sw_reset(icm);
	vTaskDelay(pdMS_TO_TICKS(250));

    // wake up sensor -> test later to see if it's really necessary
    icm20948_sleep(icm, false);
	icm20948_low_power(icm, false);
}

void acquire_icm20948(data_t *data, icm20948_device_t *icm, icm20948_agmt_t *agmt)
{
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    if (icm20948_get_agmt(icm, agmt) != ICM_20948_STAT_OK)
        ESP_LOGE(TAG_ICM, "Failed to read ICM20948");

    data->accel_x = agmt->acc.axes.x;
    data->accel_y = agmt->acc.axes.y;
    data->accel_z = agmt->acc.axes.z;
    data->rotation_x = agmt->gyr.axes.x;
    data->rotation_y = agmt->gyr.axes.y;
    data->rotation_z = agmt->gyr.axes.z;
    data->magnet_x = agmt->mag.axes.x;
    data->magnet_y = agmt->mag.axes.y;
    data->magnet_z = agmt->mag.axes.z;

    xSemaphoreGive(xI2CMutex);
    vTaskDelay(0);
}

//void init_bmp390(bmp390_config_t* dev_cfg, bmp390_handle_t* dev_hdl)
//{
//    // init device
//    bmp390_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
//    if (dev_hdl == NULL) {
//        ESP_LOGE(APP_TAG, "bmp390 handle init failed");
//        assert(dev_hdl);
//    }
//
//    /* configuration registers */
//    bmp390_power_control_register_t     power_ctrl_reg;
//    bmp390_configuration_register_t     config_reg;
//    bmp390_oversampling_register_t      oversampling_reg;
//    bmp390_output_data_rate_register_t  output_data_rate_reg;
//    bmp390_interrupt_control_register_t interrupt_ctrl_reg;
//
//    /* attempt to read configuration register */
//    bmp390_get_configuration_register(dev_hdl, &config_reg);
//
//    /* attempt to read oversampling register */
//    bmp390_get_oversampling_register(dev_hdl, &oversampling_reg);
//
//    /* attempt to read to power control register */
//    bmp390_get_power_control_register(dev_hdl, &power_ctrl_reg);
//
//    /* attempt to read to output data rate register */
//    bmp390_get_output_data_rate_register(dev_hdl, &output_data_rate_reg);
//
//    /* attempt to read to interrupt control register */
//    bmp390_get_interrupt_control_register(dev_hdl, &interrupt_ctrl_reg);
//
//
//    ESP_LOGI(APP_TAG, "Configuration (0x%02x): %s", config_reg.reg,           uint8_to_binary(config_reg.reg));
//    ESP_LOGI(APP_TAG, "Oversampling  (0x%02x): %s", oversampling_reg.reg,     uint8_to_binary(oversampling_reg.reg));
//    ESP_LOGI(APP_TAG, "Data Rate     (0x%02x): %s", output_data_rate_reg.reg, uint8_to_binary(output_data_rate_reg.reg));
//    ESP_LOGI(APP_TAG, "Power Control (0x%02x): %s", power_ctrl_reg.reg,       uint8_to_binary(power_ctrl_reg.reg));
//    ESP_LOGI(APP_TAG, "Int Control   (0x%02x): %s", interrupt_ctrl_reg.reg,   uint8_to_binary(interrupt_ctrl_reg.reg));
//
//    if(interrupt_ctrl_reg.bits.irq_data_ready_enabled)
//        ESP_LOGE(APP_TAG, "bmp390 irq data ready is enabled");
//}

//void acquire_bmp390(data_t *data, bmp390_handle_t* dev_hdl)
//{
//    ESP_LOGI(APP_TAG, "######################## BMP390 - START #########################");
//        //
//
//        // sensor readings
//        if (bmp390_get_measurements(dev_hdl, &data->temperature, &data->pressure); != ESP_OK)
//            ESP_LOGE(APP_TAG, "bmp390 device read failed (%s)", esp_err_to_name(result));
//        else {
//            pressure = pressure / 100;
//            ESP_LOGI(APP_TAG, "air temperature:     %.2f °C", temperature);
//            ESP_LOGI(APP_TAG, "barometric pressure: %.2f hPa", pressure);
//        }
//
//        // necessario revisar
//        float temp_altitude = 0;
//        BMP280 altitude calculation (barometric formula)
//        temp_altitude = 44330 * (1 - powf(data->pressure / 101325, 1 / 5.255));
//
//        if (temp_altitude > data->max_altitude)
//            data->max_altitude = temp_altitude;
//        data->bmp_altitude = temp_altitude;
//
//        //
//    ESP_LOGI(APP_TAG, "######################## BMP390 - END ###########################");
//}

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

    // ICM20948
    icm20948_device_t icm;
    icm20948_agmt_t agmt;
    init_icm20948(&icm);

    // BMP390

    // init bmp390
    //bmp390_config_t dev_cfg = I2C_BMP390_CONFIG_DEFAULT;
    //bmp390_handle_t dev_hdl;
    //init_bmp390(&dev_cfg, &dev_hdl);


    vTaskDelay(pdMS_TO_TICKS(1000));
    while (true)
    {
        // Time and status update
        data.time = (int32_t)(esp_timer_get_time() / 1000);
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        data.status = STATUS;
        xSemaphoreGive(xStatusMutex);

        // ICM20948
        acquire_icm20948(&data, &icm, &agmt);


        // BMP390
        //acquire_bmp390(&data, &dev_bmp);


        status_checks(&data);

        // Print data
        ESP_LOGI(TAG_ACQ, "\tTime: %ld, Status: %ld V: %.2f\r\n"
                          "\tBMP\t\tP: %.2f, T: %.2f, A: %.2f\r\n"
                          "\tAccel\t\tX: %.2f, Y: %.2f, Z: %.2f\r\n"
                          "\tGyro\t\tH: %.2f, P: %.2f, Y: %.2f\r\n"
                          "\tMagnet\t\tX: %.2f, Y: %.2f, Z: %.2f\r\n"
                          "\tGPS\t\tLat: %.5f, Lon: %.5f, A-GPS: %.2f",
                 data.time, data.status, data.voltage,
                 data.pressure, data.temperature, data.bmp_altitude,
                 data.accel_x, data.accel_y, data.accel_z,
                 data.rotation_x, data.rotation_y, data.rotation_z,
                 data.magnet_x, data.magnet_y, data.magnet_z,
                 data.latitude, data.longitude, data.gps_altitude);

        send_queues(&data);

        // REDUCE AFTER OPTIMIZING CODE
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    //bmp390_delete(dev_hdl);
    vTaskDelete(NULL);
}