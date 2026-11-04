#ifndef ACCEL
#define ACCEL_SENSOR_READER_H

#include "../csv_reader/csv_reader.h"
#include <stddef.h>
#include <stdint.h>

#define ACCEL_DATA_FILEPATH "data/accel.csv"

typedef struct {
    float x;
    float y;
    float z;
} vect3_t;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

typedef struct {
    uint32_t timestamp_ms;
    vect3_t clean;
    vect3_t noisy;
    quaternion_t orientation;
    vect3_t true_acceleration;
} accel_sample_t;

typedef struct {
    sensor_log_t base;
} accel_log_t;

accel_log_t *read_accel_csv();

#endif // ACCEL_SENSOR_READER_H