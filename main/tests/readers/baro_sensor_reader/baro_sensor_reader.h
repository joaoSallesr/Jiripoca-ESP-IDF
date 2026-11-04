#ifndef BARO_SENSOR_READER_H
#define BARO_SENSOR_READER_H

#include "../csv_reader/csv_reader.h"
#include <stddef.h>
#include <stdint.h>

#define BARO_DATA_FILEPATH "data/baro.csv"

typedef struct {
    uint32_t timestamp_ms;
    float    pressure_pa;
    float    altitude_z_m;
} baro_sample_t;

typedef struct {
    sensor_log_t base;
} baro_log_t;

baro_log_t *read_baro_csv();

#endif // BARO_SENSOR_READER_H