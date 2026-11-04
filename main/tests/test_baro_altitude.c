#include <stddef.h>
#include <stdio.h>

#define KNOWN_ALTITUDE 160.0f
#include "stubs/global.h"

#include "../sensors/bmp.c"
#include "readers/baro_sensor_reader/baro_sensor_reader.h"
#include "test_metrics.h"
#include "unity.h"

// The pressure fixture is noisy; these bounds cover its observed altitude error.
#define MAX_ABS_ERROR_TOLERANCE 45.0
#define RMSE_TOLERANCE          32.0
#define BIAS_TOLERANCE          32.0

static baro_log_t *baro_log = NULL;

void setUp(void) { baro_log = read_baro_csv(); }

void tearDown(void) { free_sensor_log(baro_log); }

void test_baro_reader(void) {
    TEST_ASSERT_NOT_NULL(baro_log);

    size_t count = baro_log->base.sample_count;
    TEST_ASSERT_TRUE(count > 0);

    TEST_ASSERT_TRUE(count >= CALIBRATION_SAMPLES);
    sea_temp = KNOWN_TEMPERATURE + 273.15f + medium_lapse_rate * KNOWN_ALTITUDE;

    float pressure_sum = 0.0f;
    for (size_t i = 0; i < CALIBRATION_SAMPLES; i++) {
        baro_sample_t *sample = (baro_sample_t *)baro_log->base.samples + i;
        pressure_sum += sample->pressure_pa;
    }
    float mean_pressure = pressure_sum / CALIBRATION_SAMPLES;
    sea_pressure = mean_pressure / powf(1 - KNOWN_ALTITUDE * medium_lapse_rate / sea_temp, inv_exponent);

    test_error_metrics_t metrics = TEST_ERROR_METRICS_INIT;

    for (size_t i = 0; i < count; i++) {
        baro_sample_t *current_sample = (baro_sample_t *)baro_log->base.samples + i;

        double measured_pressure   = current_sample->pressure_pa;
        double calculated_altitude = get_barometric_altitude(measured_pressure);

        double expected_altitude = current_sample->altitude_z_m;
        double error = calculated_altitude - expected_altitude;
        test_error_metrics_add(&metrics, calculated_altitude, expected_altitude);
        if (i % 100 == 0) {
            printf("Sample [%zu] | Expected: %.2f m | Actual: %.2f m | Error: %.2f m\n", i, expected_altitude, calculated_altitude, error);
        }
    }

    test_error_metrics_print("Altitude Error Summary", &metrics);
    TEST_ASSERT_ERROR_METRICS_WITHIN(&metrics, MAX_ABS_ERROR_TOLERANCE, RMSE_TOLERANCE, BIAS_TOLERANCE);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_baro_reader);
    return UNITY_END();
}