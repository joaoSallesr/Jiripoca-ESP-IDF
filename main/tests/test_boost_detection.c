#include "flight_control.c"
#include "readers/flight_log_reader/flight_log_reader.h"
#include "stubs/global.h"

#define HEADER_H

#include "unity.h"

static flight_telemetry_log_t *telemetry_log;
static flight_event_list_t    *expected_events;

void setUp(void) {
    telemetry_log   = read_flight_telemetry_from_csv();
    expected_events = read_expected_events_from_txt();
}

void tearDown(void) {
    if (telemetry_log != NULL) {
        free(telemetry_log->samples);
        free(telemetry_log);
    }

    if (expected_events != NULL) {
        for (size_t i = 0; i < expected_events->total_events; i++) {
            free((void *)expected_events->events[i].event_name);
        }
        free(expected_events->events);
        free(expected_events);
    }
}

void test_boost_phase_detection(void) {
    TEST_ASSERT_NOT_NULL(telemetry_log);
    TEST_ASSERT_NOT_NULL(expected_events);

    uint32_t expected_boost_trigger_time_ms = get_expected_event_trigger_time(expected_events, "STATE_BOOST");
    printf("[DEBUG Test] Expected STATE_BOOST trigger time from txt: %u ms\n", expected_boost_trigger_time_ms);

    data_g.flight_state = STATE_IDLE;
    boost_check(0.0f, 0.0f, 0);

    bool     is_boost_triggered           = false;
    uint32_t actual_boost_trigger_time_ms = 0;

    for (int i = 0; telemetry_log != NULL && i < telemetry_log->total_samples && !is_boost_triggered; i++) {
        flight_telemetry_sample_t current_sample        = telemetry_log->samples[i];
        uint32_t                  current_timestamp_ms  = current_sample.timestamp_ms;
        float                     vertical_acceleration = current_sample.vertical_acceleration_mps2;
        float                     vertical_velocity     = current_sample.vertical_velocity_mps;

        boost_check(vertical_acceleration, vertical_velocity, current_timestamp_ms);
        is_boost_triggered = (data_g.flight_state == STATE_BOOST);

        if (is_boost_triggered) {
            actual_boost_trigger_time_ms = current_timestamp_ms;
            printf("[DEBUG Test] BOOST triggered at sample [%d]: t=%u ms, az=%.2f m/s², vz=%.2f m/s\n", i, current_timestamp_ms,
                   vertical_acceleration, vertical_velocity);
        }
    }
    printf("Expected boost trigger time: %u ms, Actual: %u ms\n", expected_boost_trigger_time_ms, actual_boost_trigger_time_ms);
    TEST_ASSERT_EQUAL(expected_boost_trigger_time_ms, actual_boost_trigger_time_ms);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_boost_phase_detection);
    return UNITY_END();
}
