#include "baro_sensor_reader.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int baro_parse_line(const char *raw_csv_row, void *current_slot_address) {
    float time_sec   = 0.0f;
    float pressure   = 0.0f;
    float altitude_z = 0.0f;

    baro_sample_t *current_sample = (baro_sample_t *)current_slot_address;

    if (sscanf(raw_csv_row, "%f,%f,%f", &time_sec, &pressure, &altitude_z) != 3) {
        return 0;
    }
    current_sample->timestamp_ms = (uint32_t)(time_sec * 1000.0f);
    current_sample->pressure_pa = pressure;
    current_sample->altitude_z_m = altitude_z;
    return 1;
}
baro_log_t *read_baro_csv() {
    csv_reader_spec_t spec = {.filepath = BARO_DATA_FILEPATH, .sample_size = sizeof(baro_sample_t), .parse_line = baro_parse_line};

    sensor_log_t *generic_log = read_csv(&spec);
    if (generic_log == NULL) {
        return NULL;
    }
    baro_log_t *log = (baro_log_t *)generic_log;
    return log;
}
