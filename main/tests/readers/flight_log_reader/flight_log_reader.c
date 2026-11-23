#include "flight_log_reader.h"
#include <ctype.h>
#include <inttypes.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    const char *event_name;
    uint32_t    expected_trigger_time_ms;
} flight_event_t;

static flight_event_t *expected_events;
static size_t          total_expected_events;

static void free_event_array(flight_event_t *events, size_t count) {
    for (size_t i = 0; i < count; i++) {
        free((void *)events[i].event_name);
    }
    free(events);
}

void free_expected_events(void) {
    free_event_array(expected_events, total_expected_events);
    expected_events       = NULL;
    total_expected_events = 0;
}

static const char *flight_state_to_event_name(flight_state_t flight_state) {
    switch (flight_state) {
    case STATE_IDLE:
        return "IDLE";
    case STATE_BOOST:
        return "BOOST";
    case STATE_COAST:
        return "COAST";
    case STATE_DROGUE:
        return "DROGUE";
    case STATE_MAIN:
        return "MAIN";
    case STATE_LANDING:
        return "LANDING";
    case STATE_LANDED:
        return "LANDED";
    case STATE_FATAL:
        return "FATAL";
    default:
        return NULL;
    }
}

uint32_t get_expected_event_trigger_time(flight_state_t flight_state) {
    const char *target_event_name = flight_state_to_event_name(flight_state);
    if (target_event_name == NULL || expected_events == NULL) {
        return 0;
    }

    for (size_t i = 0; i < total_expected_events; i++) {
        if (strcmp(expected_events[i].event_name, target_event_name) == 0) {
            return expected_events[i].expected_trigger_time_ms;
        }
    }
    return 0;
}

bool read_expected_events_from_txt(void) {
    free_expected_events();

    FILE *events_file = fopen(EXPECTED_EVENTS_TXT_FILE_PATH, "r");
    if (events_file == NULL) {
        printf("[DEBUG] Failed to open expected events file: %s\n", EXPECTED_EVENTS_TXT_FILE_PATH);
        return false;
    }
    char            line_buffer[256];
    flight_event_t *events_array = malloc(MAX_EXPECTED_EVENTS * sizeof(flight_event_t));
    if (events_array == NULL) {
        fclose(events_file);
        return false;
    }
    size_t i = 0;
    while (i < MAX_EXPECTED_EVENTS && fgets(line_buffer, sizeof(line_buffer), events_file) != NULL) {
        char *entry = line_buffer;
        while (i < MAX_EXPECTED_EVENTS && entry != NULL) {
            char *next_entry = strchr(entry, ',');
            if (next_entry != NULL) {
                *next_entry = '\0';
            }

            char *name_start = entry;
            while (*name_start != '\0' &&
                   (isspace((unsigned char)*name_start) || *name_start == '{' || *name_start == '\'' || *name_start == '"')) {
                name_start++;
            }
            char *colon = strchr(name_start, ':');
            if (colon != NULL) {
                char *name_end = colon;
                while (name_end > name_start &&
                       (isspace((unsigned char)name_end[-1]) || name_end[-1] == '\'' || name_end[-1] == '"' || name_end[-1] == '}')) {
                    name_end--;
                }

                size_t name_length = (size_t)(name_end - name_start);
                char  *time_end    = NULL;
                float  time_sec    = strtof(colon + 1, &time_end);
                if (name_length > 0 && time_end != colon + 1) {
                    char *event_name = malloc(name_length + 1);
                    if (event_name == NULL) {
                        fclose(events_file);
                        free_event_array(events_array, i);
                        return false;
                    }
                    memcpy(event_name, name_start, name_length);
                    event_name[name_length] = '\0';
                    events_array[i].event_name               = event_name;
                    events_array[i].expected_trigger_time_ms = (uint32_t)(time_sec * 1000.0f);
                    printf("[DEBUG Event Reader] Parsed event [%zu]: name='%s', time=%.4fs (%u ms)\n", i, events_array[i].event_name,
                           time_sec, events_array[i].expected_trigger_time_ms);
                    i++;
                }
            }
            entry = next_entry != NULL ? next_entry + 1 : NULL;
        }
    }
    fclose(events_file);
    expected_events       = events_array;
    total_expected_events = i;
    printf("[DEBUG Event Reader] Total expected events loaded: %zu\n", i);
    return true;
}

flight_telemetry_log_t *read_flight_telemetry_from_csv(void) {
    FILE *telemetry_file = fopen(TELEMETRY_CSV_FILE_PATH, "r");
    if (telemetry_file == NULL) {
        printf("[DEBUG] Failed to open telemetry CSV file: %s\n", TELEMETRY_CSV_FILE_PATH);
        return NULL;
    }
    char                       line_buffer[256];
    flight_telemetry_sample_t *telemetry_samples = malloc(MAX_TELEMETRY_SAMPLES * sizeof(flight_telemetry_sample_t));
    if (telemetry_samples == NULL) {
        fclose(telemetry_file);
        return NULL;
    }
    // Skip header line
    fgets(line_buffer, sizeof(line_buffer), telemetry_file);
    int i = 0;
    while (fgets(line_buffer, sizeof(line_buffer), telemetry_file) != NULL && i < MAX_TELEMETRY_SAMPLES) {
        float time_sec = 0.0f;
        float altitude = 0.0f;
        float v_vel    = 0.0f;
        float v_acc    = 0.0f;
        if (sscanf(line_buffer, "%f,%f,%f,%f", &time_sec, &altitude, &v_vel, &v_acc) == 4) {
            telemetry_samples[i].timestamp_ms               = (uint32_t)(time_sec * 1000.0f);
            telemetry_samples[i].altitude_m                 = altitude;
            telemetry_samples[i].vertical_velocity_mps      = v_vel;
            telemetry_samples[i].vertical_acceleration_mps2 = v_acc;
            if (i < 5) {
                printf("[DEBUG Telemetry Reader] Sample [%d]: time_sec=%.6f (t=%ums), alt=%.2fm, vz=%.2fm/s, az=%.2fm/s²\n", i, time_sec,
                       telemetry_samples[i].timestamp_ms, altitude, v_vel, v_acc);
            }
            i++;
        }
    }
    fclose(telemetry_file);
    flight_telemetry_log_t *telemetry_log = malloc(sizeof(flight_telemetry_log_t));
    if (telemetry_log == NULL) {
        free(telemetry_samples);
        return NULL;
    }
    telemetry_log->samples       = telemetry_samples;
    telemetry_log->total_samples = i;
    printf("[DEBUG Telemetry Reader] Total telemetry samples loaded: %d\n", i);
    return telemetry_log;
}