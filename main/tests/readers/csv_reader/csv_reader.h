#pragma once

#include <stddef.h>
#include <stdint.h>
#define DEFAULT_INITIAL_CAPACITY 512

typedef struct {
    const char *filepath;
    size_t      sample_size;
    int (*parse_line)(const char *raw_csv_row, void *current_slot_address);
} csv_reader_spec_t;

typedef struct {
    void  *samples;
    size_t sample_count;
    size_t sample_size;
} sensor_log_t;

sensor_log_t *read_csv(const csv_reader_spec_t *spec);
void          free_sensor_log(void *log_ptr);