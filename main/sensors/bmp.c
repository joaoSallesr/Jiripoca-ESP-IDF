#include "header.h"

#define CALIBRATION_SAMPLES 20
#define BMP_TASK_DELAY_MS 20

static const char *TAG = "BMP390";

// -K/m, L: average lapse rate
static const float medium_lapse_rate = 0.0065f;
static const float hypsometric_constant = 29.271247f; // specific dry air constant/gravity

// R: universal gas constant, g: gravitational acceleration, M: dry air molar mass
static const float exponent = 0.1902665f;      // -R*L/g*M
static const float inv_exponent = 5.25578596f; // -g*M/R*L

static const float standard_sea_pressure = 101325.0f; // Pa (atmospheric preassure at sea level)
static const float standard_sea_temp = 288.15f;       // K (15°C)

static float sea_pressure = 0.0f;
static float sea_temp = 0.0f;
static float initial_temp = 0.0f;

static float get_barometric_altitude(const float pressure)
{
    float check_sea_pressure = (sea_pressure == 0) ? standard_sea_pressure : sea_pressure;
    float check_sea_temp = (sea_temp == 0) ? standard_sea_temp : sea_temp;

    float alt = (check_sea_temp / medium_lapse_rate) * (1 - powf(pressure / check_sea_pressure, exponent));
    return alt;
}

static float get_hypsometric_altitude(const float pressure, const float temperature)
{
    float check_sea_pressure = (sea_pressure == 0) ? standard_sea_pressure : sea_pressure;
    float check_sea_temp = (sea_temp == 0) ? standard_sea_temp : sea_temp;
    float mean_temperature = (temperature + check_sea_temp) / 2;

    float alt = hypsometric_constant * mean_temperature * log(check_sea_pressure / pressure);
    return alt;
}

static void bmp_init(bmp390_handle_t *bmp_hdl)
{
    // BMP390 struct setup
    bmp390_config_t bmp_cfg = {
        .i2c_address = BMP390_I2C_ADDRESS,
        .i2c_clock_speed = I2C_SPEED,
        .power_mode = BMP390_POWER_MODE_NORMAL,
        .iir_filter = BMP390_IIR_FILTER_3,
        .pressure_oversampling = BMP390_PRESSURE_OVERSAMPLING_4X,
        .temperature_oversampling = BMP390_TEMPERATURE_OVERSAMPLING_SKIPPED,
        .output_data_rate = BMP390_ODR_20MS,
    };

    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    ESP_ERROR_CHECK(bmp390_init(bus_handle, &bmp_cfg, bmp_hdl));
    xSemaphoreGive(xI2CMutex);

    ESP_LOGI(TAG, "BMP390 initialized");
    vTaskDelay(pdMS_TO_TICKS(100)); // wait for parameter changes ((1 + IIR_COEFF) * T_OS + 2.5 ms ?)
}

void bmp_task(void *pvParameters)
{
    bmp390_handle_t bmp_hdl;
    bmp_init(&bmp_hdl);
    bmp390_sample_t sample;
    static float initial_alt = 0.0f;

    // calibration samples
    float pressure_sum = 0.0f;
    float alt_sum = 0.0f;
    uint8_t pressure_samples = 0;
    uint8_t alt_samples = 0;

    // wait for a valid ICM temperature (pode ser trocado futuramente por um xEventGroupWaitBits)
    /*while (initial_temp == 0)
    {
        icm20948_sample_t icm_sample;
        portENTER_CRITICAL(&xICMMutex);
        icm_sample = icm_sample_g;
        portEXIT_CRITICAL(&xICMMutex);
        initial_temp = icm_sample.initial_temperature;
        if (initial_temp == 0)
            vTaskDelay(pdMS_TO_TICKS(10));
    }*/

    // get sea level temperature using KNOWN_ALTITUDE
    // sea_temp = initial_temp + 273.15f + (medium_lapse_rate * KNOWN_ALTITUDE);

    while (true)
    {
        xSemaphoreTake(xI2CMutex, portMAX_DELAY);
        esp_err_t err = bmp390_get_measurements(bmp_hdl, &sample.temperature, &sample.pressure);
        xSemaphoreGive(xI2CMutex);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "get measurements failed: %s", esp_err_to_name(err));
            continue;
        }

        // get sea level temperature
        if (sea_temp == 0)
            sea_temp = sample.temperature + 273.15f + (medium_lapse_rate * KNOWN_ALTITUDE);

        // calibrate sea level pressure
        if (pressure_samples < CALIBRATION_SAMPLES)
        {
            pressure_sum += sample.pressure;
            pressure_samples++;

            if (pressure_samples == CALIBRATION_SAMPLES)
            {
                float mean_pressure = pressure_sum / pressure_samples;
                float check_sea_temp = (sea_temp == 0) ? standard_sea_temp : sea_temp;
                sea_pressure = mean_pressure / powf((1 - KNOWN_ALTITUDE * medium_lapse_rate / check_sea_temp), inv_exponent);
                ESP_LOGI("BMP390", "Sea level pressure calibrated: %.2f Pa", sea_pressure);
            }

            vTaskDelay(pdMS_TO_TICKS(BMP_TASK_DELAY_MS));
            continue;
        }

        // get altitude
        //float alt = get_barometric_altitude(sample.pressure);
        float alt = get_hypsometric_altitude(sample.pressure, sample.temperature);

        // calibrate initial altitude
        if (alt_samples < CALIBRATION_SAMPLES)
        {
            alt_sum += alt;
            alt_samples++;

            if (alt_samples == CALIBRATION_SAMPLES)
            {
                initial_alt = alt_sum / alt_samples;
                ESP_LOGI("BMP390", "Initial altitude calibrated: %.2f m", initial_alt);
            }
        }

        // update global BMP sample
        portENTER_CRITICAL(&xBMPMutex);
        bmp_sample_g.pressure = sample.pressure;
        bmp_sample_g.temperature = sample.temperature;
        bmp_sample_g.altitude = alt;
        bmp_sample_g.initial_altitude = initial_alt;
        portEXIT_CRITICAL(&xBMPMutex);

        // notify acquire task that new data is available
        xTaskNotifyGive(xTaskAcquire);

        portENTER_CRITICAL(&xDATAMutex);
        bool landed = (data_g.status & LANDED);
        portEXIT_CRITICAL(&xDATAMutex);

        if (landed)
            break;

        vTaskDelay(pdMS_TO_TICKS(BMP_TASK_DELAY_MS));
    }

    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    bmp390_delete(bmp_hdl);
    xSemaphoreGive(xI2CMutex);
    vTaskDelete(NULL);
}