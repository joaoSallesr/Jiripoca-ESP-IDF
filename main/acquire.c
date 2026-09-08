#include "global.h"
#define DEFAULT_PULSE_MS 500
static const char *TAG_ACQ    = "ACQUIRE";
static const char *TAG_DEPLOY = "DEPLOY";

#define sACC_THRESHOLD 0.5f // Threshold for GPS sAcc above which xR is scaled up in eskf_update_gps

#define DEFAULT_PULSE_MS             500
#define LANDED_VELOCITY_THRESHOLD_MS 0.5f
#define LANDED_ACCEL_THRESHOLD_MS2   1.0f

static void pulse_actuator(gpio_num_t gpio, const char *log_tag, const char *log_msg, uint32_t duration_ms) {
    if (log_msg != NULL && log_tag != NULL) {
        ESP_LOGW(log_tag, "%s", log_msg);
    }

    uint32_t pulse_time = (duration_ms > 0) ? duration_ms : DEFAULT_PULSE_MS;

    gpio_set_level(gpio, HIGH);
    vTaskDelay(pdMS_TO_TICKS(pulse_time));
    gpio_set_level(gpio, LOW);
}

uint32_t track_start_time(uint32_t now, uint32_t started_at, bool condition) {
    if (!condition)
        return 0;
    return (started_at == 0) ? now : started_at;
}

static void confirm_status_transition(data_t *data, bool condition, uint32_t *started_at, uint32_t status_flag,
                                      uint32_t required_duration) {
    uint32_t now = data->time;

    *started_at = track_start_time(now, *started_at, condition);

    bool has_started = *started_at != 0;

    bool has_reached_threshold = has_started && (now - *started_at) > required_duration;

    if (has_reached_threshold) {
        data->status |= status_flag;
        *started_at = 0;
    }
}

void landing_status_check(data_t *data) {
    bool is_main_deployed = data->status & MAIN_DEPLOYED;
    bool is_landing       = data->status & LANDING;

    float current_altitude  = data->kf.x.h;
    float vertical_velocity = data->kf.x.vz;

    bool is_descending = vertical_velocity < 0.0f;

    if (!is_main_deployed || is_landing || !is_descending)
        return;

    static uint32_t landing_started_at = 0;

    float time_to_ground = current_altitude / -vertical_velocity;

    bool landing_imminent = time_to_ground < PREPARE_FOR_LANDING_S;

    confirm_status_transition(data, landing_imminent, &landing_started_at, LANDING, THRESHOLD_MS);
}

void coast_status_check(data_t *data) {
    bool is_boost = data->status & BOOST;
    bool is_coast = data->status & COAST;

    if (!is_boost || is_coast)
        return;

    static uint32_t coast_started_at = 0;

    float accel_z     = data->icm.az;
    bool  is_coasting = accel_z < COAST_THRESHOLD_A;

    confirm_status_transition(data, is_coasting, &coast_started_at, COAST, THRESHOLD_MS);
}

void boost_status_check(data_t *data) {
    bool is_boost = data->status & BOOST;

    if (is_boost)
        return;
    static uint32_t boost_started_at = 0;

    float accel_z      = data->icm.az;
    float vel_vertical = data->kf.x.vz;
    bool  is_boosting  = accel_z > BOOST_THRESHOLD_A && vel_vertical > 0.0f;

    confirm_status_transition(data, is_boosting, &boost_started_at, BOOST, THRESHOLD_MS);
}

void landed_status_check(data_t *data) {
    bool is_landing = data->status & LANDING;
    bool is_landed  = data->status & LANDED;

    if (!is_landing || is_landed)
        return;

    float vertical_speed = fabsf(data->kf.x.vz);

    bool is_at_rest = vertical_speed < LANDED_VELOCITY_THRESHOLD_MS && data->icm.accel < LANDED_ACCEL_THRESHOLD_MS2;

    static uint32_t landed_started_at = 0;

    confirm_status_transition(data, is_at_rest, &landed_started_at, LANDED, 10 * THRESHOLD_MS);
}

void main_deploy_status_check(data_t *data) {
    bool  is_drogue_deployed     = data->status & DROGUE_DEPLOYED;
    bool  is_main_deployed       = data->status & MAIN_DEPLOYED;
    float current_altitude       = data->kf.x.h;
    float vel_vertical           = data->kf.x.vz;
    bool  is_below_main_altitude = (current_altitude < MAIN_ALTITUDE);
    bool  is_descending          = (vel_vertical < 0);

    if (!is_drogue_deployed || is_main_deployed || !is_below_main_altitude || !is_descending)
        return;

    data->status |= MAIN_DEPLOYED;
    pulse_actuator(MAIN_GPIO, TAG_DEPLOY, "Main deployed", 0);
}

void drogue_deploy_status_check(data_t *data) {
    bool is_armed           = data->status & ARMED;
    bool is_coast           = data->status & COAST;
    bool is_drogue_deployed = data->status & DROGUE_DEPLOYED;

    if (!is_armed || !is_coast || is_drogue_deployed)
        return;

    float current_altitude = data->kf.x.h;
    float apogee_altitude  = data->kf.x.apogee;
    float vertical_speed   = fabsf(data->kf.x.vz);

    bool is_near_apogee = vertical_speed < DROGUE_THRESHOLD_V;

    bool            is_descending     = current_altitude < apogee_altitude;
    uint32_t        now               = data->time;
    static uint32_t apogee_started_at = 0;

    apogee_started_at          = track_start_time(now, apogee_started_at, is_near_apogee);
    bool has_started           = apogee_started_at != 0;
    bool has_reached_threshold = has_started && (now - apogee_started_at) > THRESHOLD_MS;
    bool apogee_confirmed      = has_reached_threshold;

    if (!apogee_confirmed || !is_descending)
        return;

    data->status |= DROGUE_DEPLOYED;
    apogee_started_at = 0;

    pulse_actuator(DROGUE_GPIO, TAG_DEPLOY, "Drogue deployed", 0);
}

// Contains logic for state transitions and deployments
void status_check(data_t *data) {
    // ===================== SAFE MODE =====================
    if ((data->status & ARMED) && !(data->status & BOOST) && gpio_get_level(RBF_GPIO) == LOW) {
        portENTER_CRITICAL(&xDATAMutex);
        data_g.status &= ~(ARMED);
        portEXIT_CRITICAL(&xDATAMutex);

        ESP_LOGW(TAG_ACQ, "Disarming system");
        gpio_set_level(LED_GPIO, HIGH);
        gpio_set_level(BUZZER_GPIO, HIGH);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(LED_GPIO, LOW);
        gpio_set_level(BUZZER_GPIO, LOW);
    }

    // ===================== ARMED =====================
    else if (!(data->status & ARMED) && gpio_get_level(RBF_GPIO) == HIGH) {
        portENTER_CRITICAL(&xDATAMutex);
        data_g.status |= ARMED;
        portEXIT_CRITICAL(&xDATAMutex);

        ESP_LOGW(TAG_ACQ, "System ARMED");
        for (uint8_t i = 0; i < 3; i++) {
            gpio_set_level(LED_GPIO, HIGH);
            gpio_set_level(BUZZER_GPIO, HIGH);
            vTaskDelay(pdMS_TO_TICKS(150));
            gpio_set_level(LED_GPIO, LOW);
            gpio_set_level(BUZZER_GPIO, LOW);
            vTaskDelay(pdMS_TO_TICKS(150));
        }
    }

    boost_status_check(data);
    coast_status_check(data);

    drogue_deploy_status_check(data);
    main_deploy_status_check(data);

    landing_status_check(data);
    landed_status_check(data);
}

static void pack_save_data(const data_t *data, save_t *save_data) {
    save_data->time             = data->time;
    save_data->pressure         = data->bmp.pressure;
    save_data->latitude         = data->gps.latitude;
    save_data->longitude        = data->gps.longitude;
    save_data->gps_altitude     = data->gps.altitude;
    save_data->gps_vel_vertical = data->gps.vel_vertical;
    save_data->sAcc             = data->gps.sAcc;
    save_data->fix              = data->gps.fix;
    save_data->accel_x          = data->icm.accel_x;
    save_data->accel_y          = data->icm.accel_y;
    save_data->accel_z          = data->icm.accel_z;
    save_data->gyro_x           = data->icm.gyro_x;
    save_data->gyro_y           = data->icm.gyro_y;
    save_data->gyro_z           = data->icm.gyro_z;
    save_data->mag_x            = data->icm.mag_x;
    save_data->mag_y            = data->icm.mag_y;
    save_data->mag_z            = data->icm.mag_z;
    save_data->status           = data->status;
    save_data->voltage          = data->voltage;
}

static void pack_send_data(const data_t *data, send_t *send_data) {
    send_data->time            = data->time;
    send_data->latitude        = data->gps.latitude;
    send_data->longitude       = data->gps.longitude;
    send_data->fix             = data->gps.fix;
    send_data->q1              = data->icm.q1;
    send_data->q2              = data->icm.q2;
    send_data->q3              = data->icm.q3;
    send_data->q4              = data->icm.q4;
    send_data->kf_altitude     = data->kf.x.h;
    send_data->kf_vel_vertical = data->kf.x.vz;
    send_data->kf_apogee       = data->kf.x.apogee;
    send_data->accel           = data->icm.accel;
    send_data->status          = data->status;
    send_data->voltage         = data->voltage;
}

// Sends data to SD card, LittleFS and LoRa queues
void send_queues(const data_t *data) {
    if (!(data->status & BOOST)) // If in idle state, do not save data to save resources
    {
        save_t save_data;
        pack_save_data(data, &save_data);
        xQueueSend(xB4LaunchQueue, &save_data, 0); // This queue will be only saved at landing
    } else if (!(data->status & LANDING))          // If not approaching ground
    {
        save_t save_data;
        pack_save_data(data, &save_data);

        xQueueSend(xSDQueue, &save_data, 0);                        // Send to SD card queue
        if (!atomic_load_explicit(&lfs_full, memory_order_relaxed)) // If LittleFS is not full, send to LittleFS queue
            xQueueSend(xLittleFSQueue, &save_data, 0);
    }
    send_t send_data;
    pack_send_data(data, &send_data);
    xQueueOverwrite(xLoraQueue, &send_data); // Send to LoRa queue (length 1)
}

void task_acquire(void *pvParameters) {
    uint32_t notifiedValue;
    while (true) {
        // Wait for notification from reading tasks
        xTaskNotifyWait(0,          // don't clear any bits on entry
                        UINT32_MAX, // clear all bits on exit
                        &notifiedValue, portMAX_DELAY);

        data_t data;

        portENTER_CRITICAL(&xDATAMutex);
        data = data_g;
        portEXIT_CRITICAL(&xDATAMutex);

        portENTER_CRITICAL(&xBMPMutex);
        data.bmp = bmp_sample_g;
        portEXIT_CRITICAL(&xBMPMutex);

        portENTER_CRITICAL(&xICMMutex);
        data.icm = icm_sample_g;
        portEXIT_CRITICAL(&xICMMutex);

        portENTER_CRITICAL(&xGPSMutex);
        data.gps = gps_sample_g;
        portEXIT_CRITICAL(&xGPSMutex);

        portENTER_CRITICAL(&xADCMutex);
        data.voltage = battery_voltage_g;
        portEXIT_CRITICAL(&xADCMutex);

        // Update time
        if (data.gps.utc_time > 1) // Register GPS time only once, if available (not 0 and not 1)
        {
            data.time = (uint32_t)(data.gps.utc_time);
            portENTER_CRITICAL(&xGPSMutex);
            gps_sample_g.utc_time = 1; // Set to 1 to indicate GPS time has been registered
            portEXIT_CRITICAL(&xGPSMutex);
        } else
            data.time = (uint32_t)(esp_timer_get_time() / 1000UL); // ms since boot

        status_check(&data); // Status check and update

        switch (notifiedValue) {
        case ICM_BIT:
            // Trust accelerometer less due to vibration if in boost phase
            float xQ = data.status & BOOST ? 25.0f : 1.0f;
            eskf_predict(&data.kf, data.icm.az, xQ);
            break;
        case BMP_BIT:
            if (!eskf_update_bar(&data.kf, data.bmp.altitude, 1.0f))
                ESP_LOGW(TAG_ACQ, "kf_update_bar skipped");
            break;
        case GPS_BIT:
            float sAcc = (float)data.gps.sAcc * 0.1f; // Convert back to m/s
            // Trust GPS less if speed accuracy is greater than threshold (2+sAcc-sACC_THRESHOLD)²
            float xR = (sAcc > sACC_THRESHOLD) ? (2.0f + sAcc - sACC_THRESHOLD) * (2.0f + sAcc - sACC_THRESHOLD) : 1.0f;
            if (!eskf_update_gps(&data.kf, data.gps.altitude, data.gps.vel_vertical, xR))
                ESP_LOGW(TAG_ACQ, "kf_update_gps skipped");
            break;
        case ADC_BIT:
            break;
        default:
            ESP_LOGW(TAG_ACQ, "Unknown notification received: %s", uint32_to_binary(notifiedValue));
        }

        send_queues(&data); // SD, littleFS and lora queues

        portENTER_CRITICAL(&xDATAMutex);
        data_g = data; // Update global data with latest data
        portEXIT_CRITICAL(&xDATAMutex);
    }
    bmp280_delete(dev_hdl);
    vTaskDelete(NULL);
}