#include "accel_sensor_reader.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int accel_parse_line(const char *raw_csv_row, void *current_slot_address) {
    float time_sec   = 0.0f;

    float n_x, n_y, n_z;
    float c_x, c_y, c_z;
    float qw, qx, qy, qz; // quaternion
    float true_ax, true_ay, true_az;

    accel_sample_t *current_sample = (accel_sample_t *)current_slot_address;

    int matched = sscanf(raw_csv_row, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                         &time_sec, &n_x, &n_y, &n_z, &c_x, &c_y, &c_z, &qw, &qx, &qy, &qz, &true_ax, &true_ay, &true_az);

    if (matched != 14) {
        return 0;
    }

    current_sample->timestamp_ms = (uint32_t)(time_sec * 1000.0f);
    current_sample->clean = (vect3_t){.x = c_x, .y = c_y, .z = c_z};
    current_sample->noisy = (vect3_t){.x = n_x, .y = n_y, .z = n_z};
    current_sample->orientation = (quaternion_t){.w = qw, .x = qx, .y = qy, .z = qz};
    current_sample->true_acceleration = (vect3_t){.x = true_ax, .y = true_ay, .z = true_az};

    return 1;
}
accel_log_t *read_accel_csv() {
    csv_reader_spec_t spec = {.filepath = ACCEL_DATA_FILEPATH, .sample_size = sizeof(accel_sample_t), .parse_line = accel_parse_line};

    sensor_log_t *generic_log = read_csv(&spec);
    if (generic_log == NULL) {
        return NULL;
    }
    accel_log_t *log = (accel_log_t *)generic_log;
    return log;
}
