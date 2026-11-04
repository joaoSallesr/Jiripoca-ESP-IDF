#include "flight_log_reader.h"
#include <inttypes.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint32_t get_expected_event_trigger_time(const flight_event_list_t *event_list, const char *target_event_name) {
    for (int i = 0; i < event_list->total_events; i++) {
        if (strcmp(event_list->events[i].event_name, target_event_name) == 0) {
            return event_list->events[i].expected_trigger_time_ms;
        }
    }
    return 0;
}

flight_event_list_t *read_expected_events_from_txt(void) {
    FILE *events_file = fopen(EXPECTED_EVENTS_TXT_FILE_PATH, "r");
    if (events_file == NULL) {
        printf("[DEBUG] Failed to open expected events file: %s\n", EXPECTED_EVENTS_TXT_FILE_PATH);
        return NULL;
    }
    char            line_buffer[256];
    flight_event_t *events_array = malloc(MAX_EXPECTED_EVENTS * sizeof(flight_event_t));
    if (events_array == NULL) {
        fclose(events_file);
        return NULL;
    }
    int i = 0;
    while (fgets(line_buffer, sizeof(line_buffer), events_file) != NULL) {
        char event_name_buf[256];
        float time_sec = 0.0f;
        if (sscanf(line_buffer, " %255[^:]:%f", event_name_buf, &time_sec) == 2) {
            events_array[i].event_name = strdup(event_name_buf);
            events_array[i].expected_trigger_time_ms = (uint32_t)(time_sec * 1000.0f);
            printf("[DEBUG Event Reader] Parsed event [%d]: name='%s', time=%.4fs (%u ms)\n",
                   i, events_array[i].event_name, time_sec, events_array[i].expected_trigger_time_ms);
            i++;
        }
    }
    fclose(events_file);
    flight_event_list_t *event_list = malloc(sizeof(flight_event_list_t));
    if (event_list == NULL) {
        free(events_array);
        return NULL;
    }
    event_list->events       = events_array;
    event_list->total_events = i;
    printf("[DEBUG Event Reader] Total expected events loaded: %d\n", i);
    return event_list;
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
            telemetry_samples[i].altitude_meters            = altitude;
            telemetry_samples[i].vertical_velocity_mps      = v_vel;
            telemetry_samples[i].vertical_acceleration_mps2 = v_acc;
            if (i < 5) {
                printf("[DEBUG Telemetry Reader] Sample [%d]: time_sec=%.6f (t=%ums), alt=%.2fm, vz=%.2fm/s, az=%.2fm/s²\n",
                       i, time_sec, telemetry_samples[i].timestamp_ms, altitude, v_vel, v_acc);
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
