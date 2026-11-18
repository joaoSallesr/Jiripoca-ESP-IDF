#include <math.h>
#include <stddef.h>
#include <stdio.h>

#include "stubs/global.h"
#include "test_metrics_vect3.h"
#include "../sensors/icm.c" // traz GYRO_SCALE_RAD (conversao LSB -> rad/s do firmware)
#include "readers/gyro_sensor_reader/gyro_sensor_reader.h"
#include "unity.h"

// Tolerancias (rad/s) do ruido "noisy" em relacao ao "clean"
#define MAX_ABS_ERROR_TOLERANCE 0.02
#define RMSE_TOLERANCE          0.004
#define BIAS_TOLERANCE          0.0005

#define REST_WINDOW_S          0.5    // rocket parado na rampa
#define REST_BIAS_TOLERANCE    5e-4   // rad/s
#define MAX_DRIFT_DEG          1.0    // deriva de atitude apos o voo inteiro
#define FULL_SCALE_RAD_S       (500.0 * M_PI / 180.0) // DPS_500 configurado no icm_init



static gyro_log_t *gyro_log = NULL;

void setUp(void) {
    gyro_log = read_gyro_csv();
}

void tearDown(void) {
    free_sensor_log(gyro_log);
}

static inline float lsb_to_rad(const float lsb){
    return lsb * GYRO_SCALE_RAD;
}
void test_gyro(void){
    TEST_ASSERT_NOT_NULL(gyro_log);
    TEST_ASSERT_TRUE(gyro_log->base.sample_count > 0);
    test_error_metrics_vect3_t metrics = TEST_ERROR_METRICS_VECT3_INIT;

    size_t count = gyro_log->base.sample_count;
    for (size_t i = 0; i < count; i++){
        gyro_sample_t current_sample = ((gyro_sample_t *)gyro_log->base.samples)[i];

        float actual_x = lsb_to_rad(current_sample.noisy.x);
        float actual_y = lsb_to_rad(current_sample.noisy.y);
        float actual_z = lsb_to_rad(current_sample.noisy.z);

        float expected_x = current_sample.clean.x;
        float expected_y = current_sample.clean.y;
        float expected_z = current_sample.clean.z;

        float actual[3] = {actual_x, actual_y, actual_z};
        float expected[3] = {expected_x, expected_y, expected_z};
        test_error_metrics_vect3_add(&metrics, actual, expected);
        if (i % 10000 == 0) {
            float err_z = actual[2] - expected[2];
            printf("Sample [%zu] | Expected Z: %.5f rad/s | Actual Z: %.5f rad/s | Error Z: %.5f rad/s\n",
                   i, expected[2], actual[2], err_z);
            fflush(stdout);
        }
    }
    TEST_ASSERT_ERROR_METRICS_VECT3_WITHIN(&metrics, MAX_ABS_ERROR_TOLERANCE, RMSE_TOLERANCE, BIAS_TOLERANCE);
    test_error_metrics_vect3_print(&metrics);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_gyro);
    int result = UNITY_END();
    return result;
}