#pragma once

#include "../../stubs/global.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define TELEMETRY_CSV_FILE_PATH       "data/flight_data.csv"
#define EXPECTED_EVENTS_TXT_FILE_PATH "data/event_triggers.txt"

#define MAX_TELEMETRY_SAMPLES 2000
#define MAX_EXPECTED_EVENTS   2000

typedef struct {
    uint32_t timestamp_ms;
    float    altitude_m;
    float    vertical_velocity_mps;
    float    vertical_acceleration_mps2;
} flight_telemetry_sample_t;

typedef struct {
    flight_telemetry_sample_t *samples;
    size_t                     total_samples;
} flight_telemetry_log_t;

uint32_t get_expected_event_trigger_time(flight_state_t flight_state);

bool                   read_expected_events_from_txt(void);
void                   free_expected_events(void);
flight_telemetry_log_t *read_flight_telemetry_from_csv(void);