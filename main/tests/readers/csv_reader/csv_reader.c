#include "csv_reader.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LINE_BUFFER_SIZE 256

void free_sensor_log(void *log_ptr) {
    if (log_ptr == NULL) {
        return;
    }
    sensor_log_t *log = (sensor_log_t *)log_ptr;
    if (log->samples != NULL) {
        free(log->samples);
    }
    free(log);
}

sensor_log_t *read_csv(const csv_reader_spec_t *spec) {

    FILE *file = fopen(spec->filepath, "r");
    if (file == NULL) {
        printf("[ERROR Reader] Failed to open clean CSV: %s\n", spec->filepath);
        return NULL;
    }

    size_t capacity = DEFAULT_INITIAL_CAPACITY;
    size_t count    = 0;
    void  *samples  = malloc(capacity * spec->sample_size);

    if (samples == NULL) {
        printf("[ERROR Reader] Memory allocation failed for clean samples buffer\n");
        fclose(file);
        return NULL;
    }

    char csv_line_buffer[LINE_BUFFER_SIZE];

    // SKIP HEADER
    if (!fgets(csv_line_buffer, sizeof(csv_line_buffer), file)) {
        printf("[WARN Reader] Clean CSV file is empty: %s\n", spec->filepath);
        free(samples);
        fclose(file);
        return NULL;
    }

    // READ CSV LINES LOOP
    while (fgets(csv_line_buffer, sizeof(csv_line_buffer), file) != NULL) {
        bool is_full = count >= capacity;
        if (is_full) {
            capacity *= 2;
            void *temp = realloc(samples, (capacity * spec->sample_size));
            if (temp == NULL) {
                free(samples);
                return NULL;
            }
            samples = temp;
        }
        char  *cpu_block_start        = (char *)samples;
        size_t offset_to_current_slot = (count * spec->sample_size);

        void *current_slot_address = cpu_block_start + offset_to_current_slot;

        if (spec->parse_line(csv_line_buffer, current_slot_address)) {
            count++;
        }
    }

    fclose(file);

    sensor_log_t *log = malloc(sizeof(sensor_log_t));
    if (!log) {
        printf("[ERROR Baro Reader] Memory allocation failed for clean log container\n");
        free(samples);
        return NULL;
    }

    log->samples      = samples;
    log->sample_count = count;
    log->sample_size  = spec->sample_size;

    printf("[INFO Baro Reader] Total CLEAN samples loaded: %zu\n", count);
    return log;
}