#include "unity.h"
#include "unity_test_utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ------------------------------------------------------------------ */
/* Trivial smoke test – proves Unity is wired up correctly             */
/* ------------------------------------------------------------------ */
TEST_CASE("sanity: 1 == 1", "[smoke]")
{
    TEST_ASSERT_EQUAL(1, 1);
}

TEST_CASE("sanity: true is true", "[smoke]")
{
    TEST_ASSERT_TRUE(true);
}

/* ------------------------------------------------------------------ */
/* Entry point                                                          */
/* ------------------------------------------------------------------ */
void app_main(void)
{
    unity_run_all_tests();

    /* Print summary, then idle */
    unity_run_menu(); /* waits for keypress – safe to Ctrl-] out of */
}
