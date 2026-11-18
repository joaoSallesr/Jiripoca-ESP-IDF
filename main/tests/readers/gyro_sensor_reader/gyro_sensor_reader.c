#include "gyro_sensor_reader.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int gyro_parse_line(const char *raw_csv_row, void *current_slot_address) {
    double time_s;
    float  raw_x, raw_y, raw_z;    // raw_wx, raw_wy, raw_wz (ruidoso)
    float  true_x, true_y, true_z; // true_wx, true_wy, true_wz (limpo)

    gyro_sample_t *current_sample = (gyro_sample_t *)current_slot_address;

    int matched = sscanf(raw_csv_row, "%lf,%f,%f,%f,%f,%f,%f",
                         &time_s, &raw_x, &raw_y, &raw_z, &true_x, &true_y, &true_z);

    if (matched != 7) {
        return 0;
    }

    current_sample->time_s = time_s;
    current_sample->noisy  = (vect3_t){.x = raw_x,  .y = raw_y,  .z = raw_z};
    current_sample->clean  = (vect3_t){.x = true_x, .y = true_y, .z = true_z};

    return 1;
}

static gyro_log_t *read_gyro(const char *filepath) {
    csv_reader_spec_t spec = {.filepath = filepath, .sample_size = sizeof(gyro_sample_t), .parse_line = gyro_parse_line};
    return (gyro_log_t *)read_csv(&spec);
}

gyro_log_t *read_gyro_csv(void) { return read_gyro(GYRO_FILEPATH); }
