#include <stdio.h>

#include "stubs/global.h"
#include "test_metrics_vect3.h"
#include "../sensors/icm.c"
#include "readers/accel_sensor_reader/accel_sensor_reader.h"

#include "unity.h"

// Compara a aceleracao do sensor (noisy), rotacionada para o referencial do mundo
// com a gravidade removida (rotateToWorld), contra true_a* do CSV.
// Tolerancias em m/s^2.
#define PER_SAMPLE_ERROR_TOLERANCE 20.0
#define MAX_OUTLIER_RATE          0.0001
#define RMSE_TOLERANCE             0.5
#define BIAS_TOLERANCE             0.35

static accel_log_t *accel_log = NULL;

void setUp(void) {
    accel_log = read_accel_csv();
}

void tearDown(void) {
    free_sensor_log(accel_log);
}

void test_accel_reader(void) {
    TEST_ASSERT_NOT_NULL(accel_log);
    TEST_ASSERT_TRUE(accel_log->base.sample_count > 0);
    test_error_metrics_vect3_t metrics = TEST_ERROR_METRICS_VECT3_INIT;
    size_t count = accel_log->base.sample_count;
    size_t outlier_count = 0;

    for (size_t i = 0; i < count; i++) {
        accel_sample_t sample = ((accel_sample_t *)accel_log->base.samples)[i];

        float sensor_accel[3] = {sample.noisy.x, sample.noisy.y, sample.noisy.z};
        float expected[3]     = {sample.true_acceleration.x, sample.true_acceleration.y, sample.true_acceleration.z};
        float actual[3]       = {0.0f, 0.0f, 0.0f};

        rotateToWorld(sample.orientation.w, sample.orientation.x, sample.orientation.y, sample.orientation.z,
                      sensor_accel, actual);

        test_error_metrics_vect3_add(&metrics, actual, expected);

        if (fabsf(actual[0] - expected[0]) > PER_SAMPLE_ERROR_TOLERANCE ||
            fabsf(actual[1] - expected[1]) > PER_SAMPLE_ERROR_TOLERANCE ||
            fabsf(actual[2] - expected[2]) > PER_SAMPLE_ERROR_TOLERANCE) {
            outlier_count++;
        }

        if (i % 10000 == 0) {
            printf("Sample [%zu] | Expected X/Y/Z: %.2f / %.2f / %.2f m/s^2 | Actual X/Y/Z: %.2f / %.2f / %.2f m/s^2\n", i,
                   expected[0], expected[1], expected[2], actual[0], actual[1], actual[2]);
        }
    }

    test_error_metrics_vect3_print(&metrics);
    size_t max_outlier_count = (size_t)(count * MAX_OUTLIER_RATE);
    printf("Samples over %.1f m/s^2 error: %zu (maximum allowed: %zu)\n", PER_SAMPLE_ERROR_TOLERANCE, outlier_count,
           max_outlier_count);
    TEST_ASSERT_TRUE(outlier_count <= max_outlier_count);
    TEST_ASSERT_FLOAT_WITHIN(RMSE_TOLERANCE, 0.0, test_error_metrics_vect3_rmse(&metrics));
    TEST_ASSERT_FLOAT_WITHIN(BIAS_TOLERANCE, 0.0, test_error_metrics_bias(&metrics.x));
    TEST_ASSERT_FLOAT_WITHIN(BIAS_TOLERANCE, 0.0, test_error_metrics_bias(&metrics.y));
    TEST_ASSERT_FLOAT_WITHIN(BIAS_TOLERANCE, 0.0, test_error_metrics_bias(&metrics.z));
}

void test_rotate_to_world_removes_gravity(void) {
    float sensor_acceleration[3] = {0.0f, 0.0f, G};
    float world_acceleration[3];

    rotateToWorld(1.0f, 0.0f, 0.0f, 0.0f, sensor_acceleration, world_acceleration);

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, world_acceleration[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, world_acceleration[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, world_acceleration[2]);
}

void test_rotate_to_world_applies_orientation(void) {
    const float half_angle = 0.70710678f;
    float sensor_acceleration[3] = {1.0f, 0.0f, G};
    float world_acceleration[3];

    rotateToWorld(half_angle, 0.0f, 0.0f, half_angle, sensor_acceleration, world_acceleration);

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, world_acceleration[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, world_acceleration[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, world_acceleration[2]);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_accel_reader);
    RUN_TEST(test_rotate_to_world_removes_gravity);
    RUN_TEST(test_rotate_to_world_applies_orientation);
    int result = UNITY_END();
    return result;
}
