#include "unity.h"
#include "eskf.h"

TEST_CASE("eskf_init returns INVALID_ARG on invalid parameters", "[eskf]")
{
    eskf_t eskf = {0};
    eskf.cfg.dt = 0.0f; // Invalid dt <= 0
    eskf.cfg.g = 9.81f;
    eskf.cfg.var.acc = 1.0f;
    eskf.cfg.var.bar = 1.0f;
    eskf.cfg.var.gps_h = 1.0f;
    eskf.cfg.var.gps_vz = 1.0f;
    eskf.cfg.var.ba = 1.0f;
    eskf.cfg.var.bb = 1.0f;
    eskf.cfg.var.θe = 1.0f;

    esp_err_t err = eskf_init(&eskf);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
    TEST_ASSERT_FALSE(eskf.initialized);
}

TEST_CASE("eskf_init succeeds with valid configuration", "[eskf]")
{
    eskf_t eskf = {0};
    eskf.cfg.dt = 0.01f;
    eskf.cfg.g = 9.80665f;
    eskf.cfg.var.acc = 0.1f;
    eskf.cfg.var.bar = 1.0f;
    eskf.cfg.var.gps_h = 2.0f;
    eskf.cfg.var.gps_vz = 0.5f;
    eskf.cfg.var.ba = 0.01f;
    eskf.cfg.var.bb = 0.01f;
    eskf.cfg.var.θe = 0.001f;
    eskf.cfg.idle_samples = 100;

    esp_err_t err = eskf_init(&eskf);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_TRUE(eskf.initialized);
}

TEST_CASE("eskf_predict correctly propagates altitude and velocity", "[eskf]")
{
    eskf_t eskf = {0};
    eskf.cfg.dt = 0.1f;
    eskf.cfg.g = 9.80665f;
    eskf.cfg.var.acc = 0.1f;
    eskf.cfg.var.bar = 1.0f;
    eskf.cfg.var.gps_h = 2.0f;
    eskf.cfg.var.gps_vz = 0.5f;
    eskf.cfg.var.ba = 0.01f;
    eskf.cfg.var.bb = 0.01f;
    eskf.cfg.var.θe = 0.001f;
    eskf.cfg.idle_samples = 100;

    esp_err_t err = eskf_init(&eskf);
    TEST_ASSERT_EQUAL(ESP_OK, err);

    float az = 20.0f; // 20 m/s^2 upward acceleration
    eskf_predict(&eskf, az, 1.0f);

    TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, eskf.x.vz);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, eskf.x.h);
}
