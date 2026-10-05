#include "global.h"

static const char *TAG_GPS = "GPS";

/* GPS CONFIGURATION */
#define CALIBRATION_SAMPLES 20

#define UBX_SYNC1       0xB5
#define UBX_SYNC2       0x62
#define UBX_MAX_PAYLOAD 64
#define UBX_HDR_LEN     4 // class, id, len_l, len_h
#define UBX_CK_LEN      2

#define UBX_ACK_TIMEOUT_MS 1500
#define UBX_ACK_RETRIES    3

/* GPS STRUCTURES */
typedef struct {
    uint8_t class;
    uint8_t id;
    uint8_t payload[UBX_MAX_PAYLOAD];
    uint8_t len;
} ubx_cmd_t;

typedef enum {
    ACK_NONE,
    ACK_OK,
    ACK_NAK,
} ubx_ack_t;

static struct {
    uint8_t   class;
    uint8_t   id;
    ubx_ack_t response;
} ack_state;

static const ubx_cmd_t gps_config[] = {
    /* {class, id, payload, len} */

    // CFG-MSG: disable unused NMEA sentences (class 0xF0, id, rate 0)
    {0x06, 0x01, {0xF0, 0x03, 0x00}, 3}, // GSV off
    {0x06, 0x01, {0xF0, 0x02, 0x00}, 3}, // GSA off
    {0x06, 0x01, {0xF0, 0x04, 0x00}, 3}, // RMC off
    {0x06, 0x01, {0xF0, 0x05, 0x00}, 3}, // VTG off
    {0x06, 0x01, {0xF0, 0x01, 0x00}, 3}, // GLL off

    // CFG-MSG: enable NAV-VELNED
    {0x06, 0x01, {0x01, 0x12, 0x01}, 3},

    // CFG-RATE: 200 ms, navRate 1, GPS time
    {0x06, 0x08, {GPS_SAMPLE_RATE_MS & 0xFF, GPS_SAMPLE_RATE_MS >> 8, 0x01, 0x00, 0x01, 0x00}, 6},
};

static void ubx_checksum(const uint8_t *frame, uint16_t len, uint8_t *ck_a, uint8_t *ck_b) {
    uint8_t a_sum = 0, b_sum = 0;

    for (int i = 0; i < len; i++) {
        a_sum += frame[i];
        b_sum += a_sum;
    }

    *ck_a = a_sum;
    *ck_b = b_sum;
}

static void ubx_handle_message(uint8_t class, uint8_t id, const uint8_t *payload, uint16_t len, gps_sample_t *gps) {
    /* ACK / NAK */
    if (class == 0x05 && len == 2 && (id == 0x01 || id == 0x00)) {
        if (payload[0] == ack_state.class && payload[1] == ack_state.id)
            ack_state.response = (id == 0x01) ? ACK_OK : ACK_NAK;
        return;
    }

    /* NAV-VELNED */
    if (class == 0x01 && id == 0x12 && len == 36) {
        int32_t  velD;                             // I4
        uint32_t sAcc;                             // U4
        memcpy(&velD, &payload[12], sizeof(velD)); // velD in cm/s (down positive)
        memcpy(&sAcc, &payload[28], sizeof(sAcc)); // sAcc in cm/s
        if (sAcc > 2550)
            sAcc = 2550;                                  // Constraint for uint8_t storage (255.0 m/s * 10)
        gps->vel_vertical = -(velD * 0.01f);              // m/s (up positive)
        gps->sAcc         = (uint8_t)roundf(sAcc * 0.1f); // m/s * 10
    }
}

static void ubx_parse_byte(uint8_t c, gps_sample_t *gps) {
    static enum { SYNC, HEADER, BODY } state = SYNC;

    static uint16_t sync;
    static uint8_t  frame[UBX_HDR_LEN + UBX_MAX_PAYLOAD + UBX_CK_LEN]; // GPS message buffer
    static uint16_t len, cnt;
    static uint8_t  checksum[2];

    switch (state) {
    /* Check Sync Char 1-2 */
    case SYNC:
        sync = (sync << 8) | c;
        if (sync == (UBX_SYNC1 << 8 | UBX_SYNC2)) {
            cnt   = 0;
            state = HEADER;
        }
        break;

    /* Check Message Header */
    case HEADER:
        frame[cnt++] = c;
        if (cnt == UBX_HDR_LEN) {
            len = frame[2] | (frame[3] << 8);
            if (len > UBX_MAX_PAYLOAD) {
                state = SYNC;
                break;
            }
            state = BODY;
        }
        break;

    /* Check Message Payload and Checksum */
    case BODY:
        frame[cnt++] = c;
        if (cnt == UBX_HDR_LEN + len + UBX_CK_LEN) {
            ubx_checksum(frame, UBX_HDR_LEN + len, &checksum[0], &checksum[1]);
            if (checksum[0] == frame[cnt - 2] && checksum[1] == frame[cnt - 1])
                ubx_handle_message(frame[0], frame[1], &frame[UBX_HDR_LEN], len, gps);
            state = SYNC;
        }
        break;
    }
}

static void ubx_send(uint8_t class, uint8_t id, const uint8_t *payload, uint16_t len) {
    if (len > UBX_MAX_PAYLOAD)
        return;

    uint8_t frame[2 + UBX_HDR_LEN + UBX_MAX_PAYLOAD + UBX_CK_LEN];

    frame[0] = UBX_SYNC1;
    frame[1] = UBX_SYNC2;
    frame[2] = class;
    frame[3] = id;
    frame[4] = len & 0xFF; // len_l
    frame[5] = len >> 8;   // len_h
    memcpy(&frame[6], payload, len);
    ubx_checksum(&frame[2], UBX_HDR_LEN + len, &frame[6 + len], &frame[7 + len]);
    uart_write_bytes(GPS_UART_NUM, frame, 8 + len);
}

static bool gps_send_wait_ack(uint8_t class, uint8_t id, const uint8_t *payload, uint16_t len) {
    gps_sample_t dummy = {0};
    uint8_t      buf[GPS_RX_CHUNK];

    for (int attempt = 0; attempt < UBX_ACK_RETRIES; attempt++) {
        ack_state.class    = class;
        ack_state.id       = id;
        ack_state.response = ACK_NONE;

        uart_flush_input(GPS_UART_NUM); // drop stale bytes
        ubx_send(class, id, payload, len);

        TickType_t start = xTaskGetTickCount();
        while (ack_state.response == ACK_NONE && (xTaskGetTickCount() - start) < pdMS_TO_TICKS(UBX_ACK_TIMEOUT_MS)) {
            int n = uart_read_bytes(GPS_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(20));
            for (int i = 0; i < n; i++)
                ubx_parse_byte(buf[i], &dummy);
        }

        if (ack_state.response == ACK_OK) {
            ESP_LOGD(TAG_GPS, "ACK 0x%02X 0x%02X after %lu ms", class, id, (uint32_t)((xTaskGetTickCount() - start) * portTICK_PERIOD_MS));
            return true;
        }

        if (ack_state.response == ACK_NAK) {
            ESP_LOGE(TAG_GPS, "NAK for 0x%02X 0x%02X", class, id);
            return false; // rejected
        }

        ESP_LOGW(TAG_GPS, "No ACK for 0x%02X 0x%02X (attempt %d/%d)", class, id, attempt + 1, UBX_ACK_RETRIES);
    }

    return false;
}

static bool gps_configure(void) {
    bool ok = true;
    for (size_t i = 0; i < sizeof(gps_config) / sizeof(gps_config[0]); i++) {
        const ubx_cmd_t *c = &gps_config[i];
        if (!gps_send_wait_ack(c->class, c->id, c->payload, c->len)) {
            ESP_LOGE(TAG_GPS, "Config %lu failed (0x%02X 0x%02X)", (uint32_t)i, c->class, c->id);
            ok = false;
        }
    }
    return ok;
}

static void gps_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = GPS_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_RX, GPS_TX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, GPS_BUFF_SIZE, 0, 0, NULL, 0));

    ESP_LOGI(TAG_GPS, "GPS UART initialized (baud %d)", GPS_BAUDRATE);
    vTaskDelay(pdMS_TO_TICKS(200)); // Wait for GPS to stabilize

    if (!gps_configure())
        ESP_LOGE(TAG_GPS, "GPS configuration failed");
}

void task_gps(void *pvParameters) {
    gps_init();

    char nmea_line[120];
    int  nmea_pos = 0;

    uint8_t      rx_buffer[GPS_RX_CHUNK];
    static float initial_altitude = 0;
    gps_sample_t gps              = {0};

    bool       logged_first_sentence = false;
    int        last_fix_quality      = -1;
    TickType_t last_no_fix_log       = xTaskGetTickCount();

    // Calibration samples
    float   altitude_sum     = 0.0f;
    uint8_t altitude_samples = 0;

    while (true) {
        // Read bytes from UART in chunks. The parser can handle partial data. Timeout of 20ms (max 50hz).
        int rx_len = uart_read_bytes(GPS_UART_NUM, rx_buffer, GPS_RX_CHUNK, pdMS_TO_TICKS(20));
        if (rx_len > 0) {
            for (int i = 0; i < rx_len; ++i) {
                uint8_t c = rx_buffer[i];

                /* ---------- UBX PARSER ---------- */
                ubx_parse_byte(c, &gps);

                /* ---------- NMEA PARSER ---------- */
                if (c == '$') {
                    nmea_pos              = 0;
                    nmea_line[nmea_pos++] = c;
                } else if (nmea_pos > 0) {
                    if (c == '\n') {
                        nmea_line[nmea_pos]         = '\0';
                        enum minmea_sentence_id sid = minmea_sentence_id(nmea_line, false);
                        if (!logged_first_sentence && sid != MINMEA_INVALID && sid != MINMEA_UNKNOWN) {
                            ESP_LOGI(TAG_GPS, "First valid NMEA sentence received: %s", minmea_sentence(sid));
                            logged_first_sentence = true;
                        }

                        switch (sid) {
                        case MINMEA_SENTENCE_GGA: {
                            struct minmea_sentence_gga gga;
                            if (minmea_check(nmea_line, false) && minmea_parse_gga(&gga, nmea_line)) {
                                if (gga.fix_quality != last_fix_quality) {
                                    ESP_LOGI(TAG_GPS, "GGA fix_quality changed: %d -> %d", last_fix_quality, gga.fix_quality);
                                    last_fix_quality = gga.fix_quality;
                                    gps.fix          = (uint8_t)gga.fix_quality;

                                    if (gga.fix_quality == 0) {
                                        portENTER_CRITICAL(&xGPSMutex);
                                        gps_sample_g.fix = 0;
                                        portEXIT_CRITICAL(&xGPSMutex);
                                    }
                                }

                                if (gga.fix_quality == 0) {
                                    TickType_t now = xTaskGetTickCount();
                                    if ((now - last_no_fix_log) >= pdMS_TO_TICKS(30000)) {
                                        ESP_LOGW(TAG_GPS, "No fix yet: sats=%d hdop=%.2f", gga.satellites_tracked,
                                                 minmea_tofloat(&gga.hdop));
                                        last_no_fix_log = now;
                                    }
                                }

                                else if (gga.fix_quality > 0) {
                                    if (gps.utc_time == 0) // Register GPS time once
                                        gps.utc_time = gga.time.hours * 10000 + gga.time.minutes * 100 + gga.time.seconds; // HHMMSS format

                                    gps.latitude  = minmea_tocoord(&gga.latitude);
                                    gps.longitude = minmea_tocoord(&gga.longitude);
                                    gps.altitude  = minmea_tofloat(&gga.altitude) - initial_altitude;

                                    if (initial_altitude == 0.0f) // Measure GPS initial altitude
                                    {
                                        if (altitude_samples < CALIBRATION_SAMPLES) {
                                            altitude_sum += gps.altitude;
                                            altitude_samples++;

                                            if (altitude_samples == CALIBRATION_SAMPLES) {
                                                initial_altitude = altitude_sum / altitude_samples;
                                                ESP_LOGI(TAG_GPS, "Initial altitude measured: %.2f m", initial_altitude);
                                            }
                                            break; // Set initial altitude before using GPS altitude data
                                        }
                                    }

                                    // Update global GPS sample
                                    portENTER_CRITICAL(&xGPSMutex);
                                    gps_sample_g.latitude     = gps.latitude;
                                    gps_sample_g.longitude    = gps.longitude;
                                    gps_sample_g.altitude     = gps.altitude;
                                    gps_sample_g.vel_vertical = gps.vel_vertical;
                                    gps_sample_g.utc_time     = gps.utc_time;
                                    gps_sample_g.sAcc         = gps.sAcc;
                                    gps_sample_g.fix          = gps.fix;
                                    portEXIT_CRITICAL(&xGPSMutex);

                                    gps.utc_time = 1;

                                    xTaskNotify(xTaskAcquire, GPS_BIT,
                                                eSetBits); // Notify acquire task that new data is available
                                }
                            }
                            break;
                        }

                        default:
                            break;
                        }

                        nmea_pos = 0;
                    } else if (nmea_pos < sizeof(nmea_line) - 1)
                        nmea_line[nmea_pos++] = c;
                }
            }
        }
    }
}