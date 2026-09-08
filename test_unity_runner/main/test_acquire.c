#include "acquire.c"
#include "global.h"
#include "unity.h"

typedef struct {
    const char *description;
    uint32_t    now;
    uint32_t    boost_started_at; // 0 = "hasn't started yet"
    float       az;
    float       vz;
    bool        expect_boost;
} boost_case_t;
static const boost_case_t boost_cases[] = {
    {"condition just started, not latched yet", 100, 0, 25.0f, 10.0f, false},
    {"condition held past threshold -> latches", 400, 100, 25.0f, 10.0f, true}, // 300ms > THRESHOLD_MS
    {"condition held but not long enough", 200, 100, 25.0f, 10.0f, false},      // 100ms < THRESHOLD_MS
    {"low accel never starts the timer", 400, 0, 5.0f, 10.0f, false},
};

TEST_CASE("boost_status_check", "[acquire]") {
    for (size_t i = 0; i < sizeof(boost_cases) / sizeof(boost_cases[0]); i++) {
        const boost_case_t *tc     = &boost_cases[i];
        data_t              data   = {0};
        status_timers_t     timers = {0};

        data.time               = tc->now;
        data.icm.az             = tc->az;
        data.kf.x.vz            = tc->vz;
        timers.boost_started_at = tc->boost_started_at;

        boost_status_check(&data, &timers);

        TEST_ASSERT_EQUAL_MESSAGE(tc->expect_boost, (data.status & BOOST) != 0, tc->description);
    }
}

TEST_CASE("landed_status_check", "[acquire]") {}