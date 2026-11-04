#include "unity.h"
#include "header.h"

TEST_CASE("update_t_apogee returns 0 when moving too fast", "[drogue]")
{
    data_t data = {0};
    data.time = 2000;
    data.kf.x.vz = DROGUE_THRESHOLD_V + 1.0f; // Above velocity threshold

    uint32_t result = update_t_apogee(1000, &data);
    TEST_ASSERT_EQUAL_UINT32(0, result);
}

TEST_CASE("update_t_apogee sets timestamp when t_apogee is 0 and velocity is low", "[drogue]")
{
    data_t data = {0};
    data.time = 3500;
    data.kf.x.vz = 0.1f; // Below velocity threshold

    uint32_t result = update_t_apogee(0, &data);
    TEST_ASSERT_EQUAL_UINT32(3500, result);
}

TEST_CASE("update_t_apogee preserves existing timestamp when velocity is low", "[drogue]")
{
    data_t data = {0};
    data.time = 4000;
    data.kf.x.vz = 0.1f;

    uint32_t result = update_t_apogee(3000, &data);
    TEST_ASSERT_EQUAL_UINT32(3000, result);
}

TEST_CASE("drogue_deploy_status_check does nothing when unarmed", "[drogue]")
{
    data_t data = {0};
    data.status = COAST; // Not ARMED
    data.time = 1000;
    data.kf.x.vz = 0.0f;
    data.kf.x.h = 490.0f;
    data.kf.x.apogee = 500.0f;

    drogue_deploy_status_check(&data);
    TEST_ASSERT_FALSE(data.status & DROGUE_DEPLOYED);
}

TEST_CASE("drogue_deploy_status_check deploys drogue when apogee confirmed and descending", "[drogue]")
{
    data_t data = {0};
    data.status = ARMED | COAST;
    data.time = 1000;
    data.kf.x.vz = 0.1f;
    data.kf.x.h = 490.0f;
    data.kf.x.apogee = 500.0f;

    // First call records t_apogee timestamp (1000)
    drogue_deploy_status_check(&data);
    TEST_ASSERT_FALSE(data.status & DROGUE_DEPLOYED);

    // Advance time beyond THRESHOLD_MS
    data.time = 1000 + THRESHOLD_MS + 50;
    drogue_deploy_status_check(&data);

    TEST_ASSERT_TRUE(data.status & DROGUE_DEPLOYED);
}
