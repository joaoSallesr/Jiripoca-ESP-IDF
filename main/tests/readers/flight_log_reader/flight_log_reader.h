#pragma once

#include <stddef.h>
#include <stdint.h>

#define TELEMETRY_CSV_FILE_PATH       "data/new_data.csv"
#define EXPECTED_EVENTS_TXT_FILE_PATH "data/event_triggers.txt"

#define MAX_TELEMETRY_SAMPLES 2000
#define MAX_EXPECTED_EVENTS   2000

// Representa um evento esperado do voo (ex: tempo em que um estado deve ser acionado)
typedef struct {
    const char *event_name;
    uint32_t    expected_trigger_time_ms;
} flight_event_t;

typedef struct {
    flight_event_t *events;
    size_t          total_events;
} flight_event_list_t;

// Uma amostra de dados de telemetria do voo obtida dos sensores
typedef struct {
    uint32_t timestamp_ms;
    float    altitude_meters;            // Antigo 'h' (altitude/altura)
    float    vertical_velocity_mps;      // Antigo 'vz' (velocidade no eixo vertical Z)
    float    vertical_acceleration_mps2; // Antigo 'az' (aceleração no eixo vertical Z)
} flight_telemetry_sample_t;

typedef struct {
    flight_telemetry_sample_t *samples;
    size_t                     total_samples;
} flight_telemetry_log_t;

uint32_t get_expected_event_trigger_time(const flight_event_list_t *event_list, const char *target_event_name);

flight_event_list_t    *read_expected_events_from_txt(void);
flight_telemetry_log_t *read_flight_telemetry_from_csv(void);
