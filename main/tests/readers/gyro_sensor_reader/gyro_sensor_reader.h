#ifndef GYRO_SENSOR_READER_H
#define GYRO_SENSOR_READER_H

#include "../csv_reader/csv_reader.h"
#include <stddef.h>
#include <stdint.h>

#define GYRO_FILEPATH "data/gyroscopes.csv"

// Taxa angular em rad/s (mesma unidade que o firmware manda pro VQF)
typedef struct {
    double time_s; // double de proposito: em ms/float o dt de 4.44 ms se perde
    vect3_t clean;
    vect3_t noisy;
} gyro_sample_t;

typedef struct {
    sensor_log_t base;
} gyro_log_t;

gyro_log_t *read_gyro_csv(void);

#endif // GYRO_SENSOR_READER_H
