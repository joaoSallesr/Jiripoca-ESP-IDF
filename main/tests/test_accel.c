#include "stubs/global.h"
#include "../sensors/icm.c"
#include "readers/accel_sensor_reader/accel_sensor_reader.h"
#include "unity.h"

void setUp(void) {}

void tearDown(void) {}

void test_accel_reader(void) {
    accel_log_t *accel_log = read_accel_csv();
    TEST_ASSERT_NOT_NULL(accel_log);
    TEST_ASSERT_TRUE(accel_log->base.sample_count > 0);

    accel_sample_t sample = ((accel_sample_t *)accel_log->base.samples)[0];
    free_sensor_log(accel_log);

    TEST_ASSERT_EQUAL_UINT32(0, sample.timestamp_ms);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0.71835397f, sample.orientation.w);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0.0f, sample.true_acceleration.z);
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
    return UNITY_END();
}
